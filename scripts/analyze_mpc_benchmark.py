#!/usr/bin/env python3
"""
Intent-MPC Benchmark Analyzer

Analyzes benchmark data from Intent-MPC trials and generates:
1. Statistical summary (console output)
2. CSV summary file
3. LaTeX table for comparison with DYNUS

Usage:
    # Analyze benchmark data
    python3 scripts/analyze_mpc_benchmark.py --data-dir data/benchmark_20260205_120000

    # With custom output
    python3 scripts/analyze_mpc_benchmark.py --data-dir data/benchmark_20260205_120000 \
        --output-name mpc_results \
        --latex-name intent_mpc_table.tex

    # Multiple configurations
    python3 scripts/analyze_mpc_benchmark.py --data-dir data/*/benchmark_*.csv
"""

import argparse
import glob
import sys
from pathlib import Path
from typing import Dict

import numpy as np
import pandas as pd


def load_benchmark_data(data_pattern: str) -> pd.DataFrame:
    """Load benchmark CSV files matching pattern"""

    # Handle directory input
    data_path = Path(data_pattern)
    if data_path.is_dir():
        # Look for benchmark_intent_mpc_*.csv files
        pattern = str(data_path / "benchmark_intent_mpc_*.csv")
    else:
        pattern = data_pattern

    csv_files = glob.glob(pattern)

    if not csv_files:
        print(f"ERROR: No CSV files found matching: {pattern}")
        print(f"       Make sure you're pointing to a directory with benchmark_intent_mpc_*.csv files")
        sys.exit(1)

    # Filter out summary files
    csv_files = [f for f in csv_files if 'summary' not in Path(f).name.lower()]

    if not csv_files:
        print(f"ERROR: No valid benchmark CSV files found")
        sys.exit(1)

    # Sort by modification time and get most recent
    csv_files_sorted = sorted(csv_files, key=lambda f: Path(f).stat().st_mtime)
    most_recent_file = csv_files_sorted[-1]

    print(f"Found {len(csv_files)} benchmark CSV file(s)")
    print(f"Loading most recent file: {Path(most_recent_file).name}")

    # Load data
    df = pd.read_csv(most_recent_file)
    print(f"\nTotal trials loaded: {len(df)}")

    # Check data validity
    if 'path_length' in df.columns:
        valid_data = (df['path_length'] > 0).sum()
        if valid_data == 0:
            print(f"\n⚠️  WARNING: All trials have path_length=0.0")
            print(f"   No trajectory data was collected.\n")

    return df


def compute_statistics(df: pd.DataFrame) -> dict:
    """Compute comprehensive statistics from benchmark data"""

    stats = {}

    # Total trials
    stats['total_trials'] = len(df)

    # Success metrics
    stats['success_rate'] = df['goal_reached'].mean() * 100  # percentage
    stats['timeout_rate'] = df['timeout_reached'].mean() * 100
    stats['collision_rate'] = df['collision'].mean() * 100

    # Filter successful trials
    successful = df[df['goal_reached'] == True]
    n_success = len(successful)

    if n_success == 0:
        print("WARNING: No successful trials found!")
        return stats

    stats['n_successful'] = n_success

    # Flight time
    if 'flight_travel_time' in successful.columns:
        values = successful['flight_travel_time'].dropna()
        if len(values) > 0:
            stats['flight_travel_time_min'] = values.min()
            stats['flight_travel_time_max'] = values.max()
            stats['flight_travel_time_mean'] = values.mean()
            stats['flight_travel_time_std'] = values.std()

    # Path length
    if 'path_length' in successful.columns:
        values = successful['path_length'].dropna()
        if len(values) > 0:
            stats['path_length_min'] = values.min()
            stats['path_length_max'] = values.max()
            stats['path_length_mean'] = values.mean()
            stats['path_length_std'] = values.std()

    # Path efficiency
    if 'path_efficiency' in successful.columns:
        values = successful['path_efficiency'].dropna()
        if len(values) > 0:
            stats['path_efficiency_mean'] = values.mean()
            stats['path_efficiency_std'] = values.std()

    # Velocity metrics
    if 'avg_velocity' in successful.columns:
        values = successful['avg_velocity'].dropna()
        if len(values) > 0:
            stats['avg_velocity_mean'] = values.mean()
            stats['max_velocity_mean'] = successful['max_velocity'].mean()

    # Acceleration metrics
    if 'avg_acceleration' in successful.columns:
        values = successful['avg_acceleration'].dropna()
        if len(values) > 0:
            stats['avg_acceleration_mean'] = values.mean()
            stats['max_acceleration_mean'] = successful['max_acceleration'].mean()

    # Jerk smoothness (RMS)
    if 'jerk_rms' in successful.columns:
        values = successful['jerk_rms'].dropna()
        if len(values) > 0:
            stats['jerk_rms_min'] = values.min()
            stats['jerk_rms_max'] = values.max()
            stats['jerk_rms_mean'] = values.mean()
            stats['jerk_rms_std'] = values.std()

    # Jerk integral
    if 'jerk_integral' in successful.columns:
        values = successful['jerk_integral'].dropna()
        if len(values) > 0:
            stats['jerk_integral_min'] = values.min()
            stats['jerk_integral_max'] = values.max()
            stats['jerk_integral_mean'] = values.mean()
            stats['jerk_integral_std'] = values.std()

    # Constraint violations (rates among successful trials)
    for viol_type in ['vel', 'acc', 'jerk']:
        col = f'{viol_type}_violation_count'
        if col in successful.columns:
            # Rate of trials with violations
            viol_rate = (successful[col] > 0).mean() * 100
            stats[f'{viol_type}_violation_rate'] = viol_rate

            # Average count when violations occur
            trials_with_viol = successful[successful[col] > 0]
            if len(trials_with_viol) > 0:
                stats[f'{viol_type}_violation_avg_count'] = trials_with_viol[col].mean()

            # Max violation value
            max_col = f'{viol_type}_violation_max'
            if max_col in successful.columns:
                stats[f'{viol_type}_violation_max_mean'] = successful[max_col].mean()

    # Collision metrics
    if 'collision_count' in df.columns:
        stats['collision_count_total'] = df['collision_count'].sum()
        stats['collision_count_mean'] = df['collision_count'].mean()

        # Collision-free ratio
        stats['collision_free_rate'] = (df['collision_count'] == 0).mean() * 100

        # Among trials with collisions
        trials_with_coll = df[df['collision_count'] > 0]
        if len(trials_with_coll) > 0:
            stats['collision_penetration_max_avg'] = trials_with_coll['collision_penetration_max'].mean()
            if 'collision_unique_obstacles' in trials_with_coll.columns:
                stats['collision_unique_obstacles_avg'] = trials_with_coll['collision_unique_obstacles'].mean()

    # Minimum distance to obstacles
    if 'min_distance_to_obstacles' in df.columns:
        values = df['min_distance_to_obstacles'].dropna()
        # Filter out inf values
        values = values[values != float('inf')]
        if len(values) > 0:
            stats['min_distance_to_obstacles_min'] = values.min()
            stats['min_distance_to_obstacles_max'] = values.max()
            stats['min_distance_to_obstacles_mean'] = values.mean()
            stats['min_distance_to_obstacles_std'] = values.std()
        else:
            stats['min_distance_to_obstacles_mean'] = 'N/A'
    else:
        stats['min_distance_to_obstacles_mean'] = 'N/A (not tracked)'

    # Replanning metrics
    if 'num_replans' in successful.columns:
        values = successful['num_replans'].dropna()
        if len(values) > 0:
            stats['num_replans_mean'] = values.mean()
            stats['num_replans_std'] = values.std()

    if 'avg_replanning_time' in successful.columns:
        values = successful['avg_replanning_time'].dropna()
        if len(values) > 0:
            stats['avg_replanning_time_mean'] = values.mean()
            stats['max_replanning_time_mean'] = successful['max_replanning_time'].mean()

    return stats


def print_statistics(stats: dict):
    """Print statistics in formatted way"""

    print("\n" + "="*80)
    print("INTENT-MPC BENCHMARK ANALYSIS RESULTS")
    print("="*80)

    print(f"\n{'OVERVIEW':-^80}")
    print(f"  Total trials: {stats.get('total_trials', 0)}")
    print(f"  Successful trials: {stats.get('n_successful', 0)}")
    print(f"  Success rate: {stats.get('success_rate', 0):.1f}%")
    print(f"  Timeout rate: {stats.get('timeout_rate', 0):.1f}%")
    print(f"  Collision rate: {stats.get('collision_rate', 0):.1f}%")

    print(f"\n{'PERFORMANCE METRICS':-^80}")
    if 'flight_travel_time_mean' in stats:
        print(f"  Travel Time:")
        print(f"    Min: {stats.get('flight_travel_time_min', 0):.2f} s")
        print(f"    Max: {stats.get('flight_travel_time_max', 0):.2f} s")
        print(f"    Mean: {stats.get('flight_travel_time_mean', 0):.2f} s ± {stats.get('flight_travel_time_std', 0):.2f}")

    if 'path_length_mean' in stats:
        print(f"  Path Length:")
        print(f"    Min: {stats.get('path_length_min', 0):.2f} m")
        print(f"    Max: {stats.get('path_length_max', 0):.2f} m")
        print(f"    Mean: {stats.get('path_length_mean', 0):.2f} m ± {stats.get('path_length_std', 0):.2f}")

    if 'path_efficiency_mean' in stats:
        print(f"  Path Efficiency: {stats['path_efficiency_mean']:.3f} ± {stats.get('path_efficiency_std', 0):.3f}")

    print(f"\n{'VELOCITY AND ACCELERATION':-^80}")
    if 'avg_velocity_mean' in stats:
        print(f"  Average velocity: {stats['avg_velocity_mean']:.2f} m/s")
        print(f"  Maximum velocity: {stats['max_velocity_mean']:.2f} m/s")

    if 'avg_acceleration_mean' in stats:
        print(f"  Average acceleration: {stats['avg_acceleration_mean']:.2f} m/s²")
        print(f"  Maximum acceleration: {stats['max_acceleration_mean']:.2f} m/s²")

    print(f"\n{'SMOOTHNESS METRICS':-^80}")
    if 'jerk_rms_mean' in stats:
        print(f"  Jerk RMS:")
        print(f"    Min: {stats.get('jerk_rms_min', 0):.2f} m/s³")
        print(f"    Max: {stats.get('jerk_rms_max', 0):.2f} m/s³")
        print(f"    Mean: {stats.get('jerk_rms_mean', 0):.2f} m/s³ ± {stats.get('jerk_rms_std', 0):.2f}")

    if 'jerk_integral_mean' in stats:
        print(f"  Jerk Integral:")
        print(f"    Min: {stats.get('jerk_integral_min', 0):.2f}")
        print(f"    Max: {stats.get('jerk_integral_max', 0):.2f}")
        print(f"    Mean: {stats['jerk_integral_mean']:.2f} ± {stats.get('jerk_integral_std', 0):.2f}")

    print(f"\n{'CONSTRAINT VIOLATIONS':-^80}")
    for viol_type in ['vel', 'acc', 'jerk']:
        rate_key = f'{viol_type}_violation_rate'
        if rate_key in stats:
            label = viol_type.upper()
            rate = stats[rate_key]
            print(f"  {label} Violation Rate: {rate:.1f}%")

            avg_count_key = f'{viol_type}_violation_avg_count'
            if avg_count_key in stats:
                print(f"    Avg violations when occurs: {stats[avg_count_key]:.1f}")

            max_key = f'{viol_type}_violation_max_mean'
            if max_key in stats:
                print(f"    Avg max value: {stats[max_key]:.2f}")

    print(f"\n{'COLLISION METRICS':-^80}")
    print(f"  Collision-free rate: {stats.get('collision_free_rate', 0):.1f}%")
    print(f"  Total collision events: {stats.get('collision_count_total', 0):.0f}")
    print(f"  Mean collisions per trial: {stats.get('collision_count_mean', 0):.2f}")
    if 'collision_penetration_max_avg' in stats:
        print(f"  Avg max penetration (when collisions occur): {stats['collision_penetration_max_avg']:.4f} m")
    if 'collision_unique_obstacles_avg' in stats:
        print(f"  Avg unique obstacles hit: {stats['collision_unique_obstacles_avg']:.1f}")

    print(f"\n{'MINIMUM DISTANCE TO OBSTACLES':-^80}")
    if 'min_distance_to_obstacles_mean' in stats:
        mean_val = stats['min_distance_to_obstacles_mean']
        if isinstance(mean_val, str):
            print(f"  {mean_val}")
        else:
            print(f"  Min: {stats.get('min_distance_to_obstacles_min', 0):.3f} m")
            print(f"  Max: {stats.get('min_distance_to_obstacles_max', 0):.3f} m")
            print(f"  Mean: {stats.get('min_distance_to_obstacles_mean', 0):.3f} m ± {stats.get('min_distance_to_obstacles_std', 0):.3f}")
    else:
        print(f"  N/A (not tracked)")

    print(f"\n{'REPLANNING METRICS':-^80}")
    if 'num_replans_mean' in stats:
        print(f"  Average replans per trial: {stats['num_replans_mean']:.1f} ± {stats.get('num_replans_std', 0):.1f}")
    if 'avg_replanning_time_mean' in stats:
        print(f"  Avg replanning time: {stats['avg_replanning_time_mean']:.2f} ms")
        print(f"  Max replanning time: {stats['max_replanning_time_mean']:.2f} ms")

    print("\n" + "="*80 + "\n")


def save_statistics_csv(stats: dict, output_path: Path):
    """Save statistics to CSV file"""

    # Create DataFrame from stats
    df = pd.DataFrame([stats])

    # Save to CSV
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_path, index=False)

    print(f"✓ Statistics saved to CSV: {output_path}")


def generate_latex_table(stats: dict, algorithm_name: str = "Intent-MPC") -> str:
    """Generate LaTeX table row for Intent-MPC (compatible with DYNUS format)"""

    # Data row only (to be inserted into existing DYNUS table)
    success_rate = stats.get('success_rate', 0)
    collision_free_rate = stats.get('collision_free_rate', 0)
    per_opt_time = stats.get('avg_replanning_time_mean', 0)
    travel_time = stats.get('flight_travel_time_mean', 0)
    path_length = stats.get('path_length_mean', 0)
    jerk_integral = stats.get('jerk_integral_mean', 0)
    min_distance = stats.get('min_distance_to_obstacles_mean', 0)
    vel_viol = stats.get('vel_violation_rate', 0)
    acc_viol = stats.get('acc_violation_rate', 0)
    jerk_viol = stats.get('jerk_violation_rate', 0)

    # Format min_distance properly
    if isinstance(min_distance, str):
        min_dist_str = min_distance
    else:
        min_dist_str = f"{min_distance:.3f}"

    # Generate the row
    latex_row = (f"      {algorithm_name} & {success_rate:.1f} & {collision_free_rate:.1f} & {per_opt_time:.1f} & "
                 f"{travel_time:.1f} & {path_length:.1f} & {jerk_integral:.1f} & {min_dist_str} & "
                 f"{vel_viol:.1f} & {acc_viol:.1f} & {jerk_viol:.1f} \\\\")

    return latex_row


def update_dynus_latex_table(stats: dict, dynus_table_path: Path, algorithm_name: str = "I-MPC"):
    """Update existing DYNUS LaTeX table by adding Intent-MPC row"""

    if not dynus_table_path.exists():
        print(f"  Warning: DYNUS table not found at {dynus_table_path}")
        print(f"  Skipping LaTeX table update")
        return None

    # Read existing table
    with open(dynus_table_path, 'r') as f:
        lines = f.readlines()

    # Generate Intent-MPC row
    impc_row = generate_latex_table(stats, algorithm_name)

    # Find the DYNUS data row and insert Intent-MPC row after it
    new_lines = []
    inserted = False

    for i, line in enumerate(lines):
        new_lines.append(line)

        # Look for DYNUS row (contains "DYNUS &" and is before \bottomrule)
        if "DYNUS &" in line and not inserted:
            # Check if I-MPC row already exists
            if i + 1 < len(lines) and ("I-MPC" in lines[i + 1] or "Intent-MPC" in lines[i + 1]):
                # Replace existing I-MPC row
                new_lines.append(impc_row + "\n")
                inserted = True
                # Skip the old I-MPC row
                continue
            else:
                # Add new I-MPC row after DYNUS
                new_lines.append(impc_row + "\n")
                inserted = True

    if not inserted:
        print(f"  Warning: Could not find DYNUS row in table")
        print(f"  Table may not be in expected format")
        return None

    # Write updated table
    with open(dynus_table_path, 'w') as f:
        f.writelines(new_lines)

    print(f"✓ Updated DYNUS LaTeX table: {dynus_table_path}")
    print(f"  Added/updated {algorithm_name} row")

    return dynus_table_path


def main():
    parser = argparse.ArgumentParser(
        description='Analyze Intent-MPC benchmark data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument(
        '--data-dir',
        type=str,
        required=True,
        help='Path to benchmark data directory or CSV file pattern'
    )

    parser.add_argument(
        '--output-name',
        type=str,
        default='mpc_benchmark_summary',
        help='Output filename prefix (default: mpc_benchmark_summary)'
    )

    parser.add_argument(
        '--latex-name',
        type=str,
        default='intent_mpc_benchmark.tex',
        help='LaTeX table filename (default: intent_mpc_benchmark.tex)'
    )

    parser.add_argument(
        '--algorithm-name',
        type=str,
        default='I-MPC',
        help='Algorithm name for LaTeX table (default: I-MPC)'
    )

    parser.add_argument(
        '--dynus-table-path',
        type=str,
        default='/home/kkondo/paper_writing/DYNUS_v3/tables/dynamic_benchmark.tex',
        help='Path to DYNUS LaTeX table to update (default: /home/kkondo/paper_writing/DYNUS_v3/tables/dynamic_benchmark.tex)'
    )

    parser.add_argument(
        '--no-update-dynus-table',
        action='store_true',
        help='Do not update DYNUS table, only generate standalone table'
    )

    args = parser.parse_args()

    # Load data
    print("="*80)
    print("INTENT-MPC BENCHMARK ANALYZER")
    print("="*80)
    print(f"\nLoading data from: {args.data_dir}\n")

    df = load_benchmark_data(args.data_dir)

    # Compute statistics
    print("Computing statistics...")
    stats = compute_statistics(df)

    # Print results
    print_statistics(stats)

    # Determine output directory
    data_path = Path(args.data_dir)
    if data_path.is_dir():
        output_dir = data_path
    else:
        output_dir = data_path.parent

    # Save CSV
    csv_output = output_dir / f"{args.output_name}.csv"
    save_statistics_csv(stats, csv_output)

    # Update DYNUS LaTeX table (unless disabled)
    print("\nGenerating LaTeX table...")

    generated_files = [csv_output]

    if not args.no_update_dynus_table:
        dynus_table_path = Path(args.dynus_table_path)
        result = update_dynus_latex_table(stats, dynus_table_path, args.algorithm_name)
        if result:
            generated_files.append(result)
            print(f"  View updated table: {result}")
    else:
        # Generate standalone table as before
        latex_row = generate_latex_table(stats, args.algorithm_name)
        latex_output = output_dir / args.latex_name

        # Create standalone table
        latex_standalone = [
            "\\begin{table*}",
            "  \\caption{Intent-MPC benchmarking results (standalone).}",
            "  \\label{tab:intent_mpc_benchmark}",
            "  \\centering",
            "  \\renewcommand{\\arraystretch}{1.2}",
            "  \\resizebox{\\textwidth}{!}{",
            "    \\begin{tabular}{c c c c c c c c c c c}",
            "      \\toprule",
            "      \\multirow{2}{*}[-0.4ex]{\\textbf{Algorithm}}",
            "      & \\multicolumn{2}{c}{\\textbf{Success}}",
            "      & \\multicolumn{1}{c}{\\textbf{Comp. Time}}",
            "      & \\multicolumn{3}{c}{\\textbf{Performance}}",
            "      & \\multicolumn{1}{c}{\\textbf{Safety}}",
            "      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}",
            "      \\\\",
            "      \\cmidrule(lr){2-3}",
            "      \\cmidrule(lr){4-4}",
            "      \\cmidrule(lr){5-7}",
            "      \\cmidrule(lr){8-8}",
            "      \\cmidrule(lr){9-11}",
            "      &",
            "      $R^{\\mathrm{succ}}$ [\\%] &",
            "      $R^{\\mathrm{coll}}_{\\mathrm{free}}$ [\\%] &",
            "      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &",
            "      $T_{\\mathrm{trav}}$ [s] &",
            "      $L_{\\mathrm{path}}$ [m] &",
            "      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &",
            "      $d_{\\mathrm{min}}$ [m] &",
            "      $\\rho_{\\mathrm{vel}}$ [\\%] &",
            "      $\\rho_{\\mathrm{acc}}$ [\\%] &",
            "      $\\rho_{\\mathrm{jerk}}$ [\\%]",
            "      \\\\",
            "      \\midrule",
            f"{latex_row}",
            "      \\bottomrule",
            "    \\end{tabular}",
            "  }",
            "  \\vspace{-1.0em}",
            "\\end{table*}"
        ]

        latex_output.write_text("\n".join(latex_standalone))
        print(f"✓ Standalone LaTeX table saved to: {latex_output}")
        generated_files.append(latex_output)

    # Summary
    print("\n" + "="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)
    print(f"\nGenerated files:")
    for i, f in enumerate(generated_files, 1):
        print(f"  {i}. {f}")
    print()


if __name__ == '__main__':
    main()
