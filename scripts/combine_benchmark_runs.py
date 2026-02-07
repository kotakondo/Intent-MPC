#!/usr/bin/env python3
"""
Combine Multiple Benchmark Runs

Combines data from multiple benchmark runs and generates aggregate statistics.
Useful for:
1. Testing reproducibility (same seeds across runs)
2. Collecting more data (different seeds across runs)

Usage:
    # Combine all runs in data/ directory
    python3 scripts/combine_benchmark_runs.py --data-dir data

    # Combine specific runs
    python3 scripts/combine_benchmark_runs.py \
        --run-dirs data/benchmark_20260205_120000 data/benchmark_20260205_130000 data/benchmark_20260205_140000

    # Analyze combined data
    python3 scripts/analyze_mpc_benchmark.py --data-dir data/combined/combined_YYYYMMDD_HHMMSS
"""

import argparse
import glob
import sys
from pathlib import Path
from datetime import datetime
import pandas as pd
import numpy as np


def find_benchmark_csv(run_dir: Path):
    """Find the benchmark CSV file in a run directory"""
    # Look for benchmark_intent_mpc_*.csv files (not summary files)
    csv_files = list(run_dir.glob("benchmark_intent_mpc_*.csv"))
    csv_files = [f for f in csv_files if 'summary' not in f.name.lower()]

    if not csv_files:
        print(f"  Warning: No benchmark CSV found in {run_dir}")
        return None

    # Get most recent if multiple
    csv_files_sorted = sorted(csv_files, key=lambda f: f.stat().st_mtime)
    return csv_files_sorted[-1]


def load_all_runs(run_dirs):
    """Load data from all run directories"""
    all_dfs = []
    run_info = []

    print(f"\nLoading data from {len(run_dirs)} run(s)...\n")

    for i, run_dir in enumerate(run_dirs):
        csv_file = find_benchmark_csv(run_dir)
        if csv_file is None:
            continue

        df = pd.read_csv(csv_file)

        # Add metadata columns
        df['run_id'] = i
        df['run_timestamp'] = run_dir.name.replace('benchmark_', '')

        # Adjust trial_id to be globally unique across runs
        df['original_trial_id'] = df['trial_id']
        df['trial_id'] = df['trial_id'] + (i * 1000)  # Offset by run

        all_dfs.append(df)

        run_info.append({
            'run_id': i,
            'run_dir': str(run_dir),
            'timestamp': run_dir.name.replace('benchmark_', ''),
            'num_trials': len(df),
            'csv_file': str(csv_file)
        })

        print(f"  Run {i}: {run_dir.name}")
        print(f"    Trials: {len(df)}")
        print(f"    Seeds: {df['seed'].min()}-{df['seed'].max()}")
        print(f"    Success rate: {df['goal_reached'].mean()*100:.1f}%")
        print()

    if not all_dfs:
        print("ERROR: No data loaded from any run directory")
        sys.exit(1)

    # Combine all dataframes
    combined_df = pd.concat(all_dfs, ignore_index=True)

    print(f"Combined {len(combined_df)} trials from {len(run_info)} run(s)")

    return combined_df, run_info


def check_reproducibility(combined_df, run_info):
    """Check if runs with same seeds produce identical results"""

    print("\n" + "="*80)
    print("REPRODUCIBILITY CHECK")
    print("="*80)

    if len(run_info) < 2:
        print("  Need at least 2 runs to check reproducibility")
        return

    # Group by seed to find duplicate seeds across runs
    seed_groups = combined_df.groupby('seed')

    duplicate_seeds = []
    for seed, group in seed_groups:
        if len(group['run_id'].unique()) > 1:
            duplicate_seeds.append((seed, group))

    if not duplicate_seeds:
        print("  No duplicate seeds found across runs")
        print("  All runs used different seed ranges (good for statistical diversity)")
        return

    print(f"  Found {len(duplicate_seeds)} seed(s) appearing in multiple runs")
    print()

    # Check if results are identical for duplicate seeds
    mismatches = []

    for seed, group in duplicate_seeds[:5]:  # Check first 5 for brevity
        print(f"  Seed {seed}:")

        # Compare key metrics
        metrics = ['goal_reached', 'collision', 'flight_travel_time', 'path_length']

        for metric in metrics:
            values = group[metric].values
            if len(values) > 1:
                if metric in ['goal_reached', 'collision']:
                    # Boolean comparison
                    all_same = all(v == values[0] for v in values)
                else:
                    # Numerical comparison (allow small floating point differences)
                    all_same = np.allclose(values, values[0], rtol=1e-3, atol=1e-6)

                status = "✓ MATCH" if all_same else "✗ MISMATCH"
                print(f"    {metric}: {values} {status}")

                if not all_same:
                    mismatches.append((seed, metric, values))

    print()

    if mismatches:
        print(f"  ⚠️  Found {len(mismatches)} mismatch(es)")
        print("  System may not be fully deterministic")
        print("  Possible causes:")
        print("    - Race conditions in multi-threaded code")
        print("    - Uninitialized random number generators")
        print("    - Floating point rounding differences")
    else:
        print("  ✓ All duplicate seeds produce identical results")
        print("  System is deterministic")


def save_combined_data(combined_df, run_info, output_dir: Path):
    """Save combined data and metadata"""

    output_dir.mkdir(parents=True, exist_ok=True)

    # Save combined CSV
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = output_dir / f"benchmark_intent_mpc_{timestamp}.csv"
    combined_df.to_csv(csv_file, index=False)
    print(f"\n✓ Combined CSV saved to: {csv_file}")

    # Save run metadata
    import json
    metadata_file = output_dir / f"run_metadata_{timestamp}.json"
    with open(metadata_file, 'w') as f:
        json.dump({
            'num_runs': len(run_info),
            'total_trials': len(combined_df),
            'timestamp': timestamp,
            'runs': run_info
        }, f, indent=2)
    print(f"✓ Metadata saved to: {metadata_file}")

    return csv_file


def print_combined_summary(combined_df, run_info):
    """Print summary statistics for combined data"""

    print("\n" + "="*80)
    print("COMBINED DATA SUMMARY")
    print("="*80)

    print(f"\nRuns: {len(run_info)}")
    print(f"Total trials: {len(combined_df)}")
    print(f"Seed range: {combined_df['seed'].min()} - {combined_df['seed'].max()}")

    print(f"\n{'SUCCESS METRICS':-^80}")
    print(f"  Success rate: {combined_df['goal_reached'].mean()*100:.1f}%")
    print(f"  Collision rate: {combined_df['collision'].mean()*100:.1f}%")
    print(f"  Timeout rate: {combined_df['timeout_reached'].mean()*100:.1f}%")

    # Per-run breakdown
    print(f"\n{'PER-RUN BREAKDOWN':-^80}")
    for run_id in sorted(combined_df['run_id'].unique()):
        run_data = combined_df[combined_df['run_id'] == run_id]
        success_rate = run_data['goal_reached'].mean() * 100
        print(f"  Run {run_id}: {len(run_data)} trials, {success_rate:.1f}% success")

    # Successful trials only
    successful = combined_df[combined_df['goal_reached'] == True]

    if len(successful) > 0:
        print(f"\n{'PERFORMANCE (SUCCESSFUL TRIALS ONLY)':-^80}")

        if 'flight_travel_time' in successful.columns:
            print(f"  Flight time: {successful['flight_travel_time'].mean():.2f} ± {successful['flight_travel_time'].std():.2f} s")

        if 'path_length' in successful.columns:
            print(f"  Path length: {successful['path_length'].mean():.2f} ± {successful['path_length'].std():.2f} m")

        if 'path_efficiency' in successful.columns:
            print(f"  Path efficiency: {successful['path_efficiency'].mean():.3f} ± {successful['path_efficiency'].std():.3f}")

        if 'jerk_integral' in successful.columns:
            print(f"  Jerk integral: {successful['jerk_integral'].mean():.2f} ± {successful['jerk_integral'].std():.2f}")

    print()


def main():
    parser = argparse.ArgumentParser(
        description='Combine multiple benchmark runs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    group = parser.add_mutually_exclusive_group(required=True)

    group.add_argument(
        '--data-dir',
        type=str,
        help='Data directory containing multiple benchmark_* subdirectories'
    )

    group.add_argument(
        '--run-dirs',
        type=str,
        nargs='+',
        help='Specific run directories to combine'
    )

    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Output directory for combined data (default: data/combined/combined_TIMESTAMP)'
    )

    parser.add_argument(
        '--check-reproducibility',
        action='store_true',
        help='Check if runs with same seeds produce identical results'
    )

    args = parser.parse_args()

    print("="*80)
    print("COMBINE BENCHMARK RUNS")
    print("="*80)

    # Find run directories
    if args.data_dir:
        data_dir = Path(args.data_dir)
        if not data_dir.exists():
            print(f"ERROR: Data directory not found: {data_dir}")
            sys.exit(1)

        # Find all benchmark_* subdirectories
        run_dirs = sorted([d for d in data_dir.glob("benchmark_*") if d.is_dir()])

        if not run_dirs:
            print(f"ERROR: No benchmark_* directories found in {data_dir}")
            sys.exit(1)

        print(f"\nFound {len(run_dirs)} benchmark run(s) in {data_dir}")
    else:
        run_dirs = [Path(d) for d in args.run_dirs]

        # Verify all directories exist
        for d in run_dirs:
            if not d.exists():
                print(f"ERROR: Directory not found: {d}")
                sys.exit(1)

        print(f"\nCombining {len(run_dirs)} specified run(s)")

    # Load all data
    combined_df, run_info = load_all_runs(run_dirs)

    # Check reproducibility if requested
    if args.check_reproducibility:
        check_reproducibility(combined_df, run_info)

    # Print summary
    print_combined_summary(combined_df, run_info)

    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if args.data_dir:
            output_dir = Path(args.data_dir) / "combined" / f"combined_{timestamp}"
        else:
            output_dir = Path("data/combined") / f"combined_{timestamp}"

    # Save combined data
    csv_file = save_combined_data(combined_df, run_info, output_dir)

    # Instructions for analysis
    print("\n" + "="*80)
    print("NEXT STEPS")
    print("="*80)
    print(f"\nAnalyze combined data:")
    print(f"  python3 scripts/analyze_mpc_benchmark.py --data-dir {output_dir}")
    print()


if __name__ == '__main__':
    main()
