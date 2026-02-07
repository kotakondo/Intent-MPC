#!/usr/bin/env python3
"""
Complete Intent-MPC Benchmark Analysis

Single script that:
1. Post-processes collisions from rosbags (if available)
2. Computes comprehensive statistics
3. Updates DYNUS LaTeX table
4. Generates summary CSV

Usage:
    python3 scripts/analyze_benchmark.py --data-dir data/benchmark_20260205_143512
"""

import argparse
import sys
from pathlib import Path
import pandas as pd
import numpy as np
import math

# Try to import rosbag for collision post-processing
try:
    import rosbag
    HAS_ROSBAG = True
except ImportError:
    HAS_ROSBAG = False
    print("Warning: rosbag not available. Will skip collision post-processing.")
    print("Install with: pip install --extra-index-url https://rospypi.github.io/simple/ rosbag")


# ============================================================================
# COLLISION POST-PROCESSING (from rosbags)
# ============================================================================

class BoundingBox:
    """Axis-aligned bounding box for collision detection"""
    def __init__(self, min_x, max_x, min_y, max_y, min_z, max_z):
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.min_z, self.max_z = min_z, max_z

    @classmethod
    def from_center_and_half_extents(cls, cx, cy, cz, hx, hy, hz):
        return cls(cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz)

    def intersects(self, other):
        return not (self.max_x < other.min_x or self.min_x > other.max_x or
                    self.max_y < other.min_y or self.min_y > other.max_y or
                    self.max_z < other.min_z or self.min_z > other.max_z)

    def distance_to(self, other):
        if self.intersects(other):
            return 0.0
        dx = max(0.0, max(self.min_x, other.min_x) - min(self.max_x, other.max_x))
        dy = max(0.0, max(self.min_y, other.min_y) - min(self.max_y, other.max_y))
        dz = max(0.0, max(self.min_z, other.min_z) - min(self.max_z, other.max_z))
        return math.sqrt(dx*dx + dy*dy + dz*dz)


def parse_obstacle_size(name: str):
    """Parse obstacle size from name (format: obstacle_dXXX_YYY_ZZZ)"""
    if len(name) == 21 and name.startswith("obstacle_d"):
        try:
            x_str, y_str, z_str = name[10:13], name[14:17], name[18:21]
            x_size = float(x_str) / 100.0
            y_size = float(y_str) / 100.0
            z_size = float(z_str) / 100.0
            return (x_size / 2.0, y_size / 2.0, z_size / 2.0)
        except:
            pass
    return (0.4, 0.4, 0.4)


def interpolate_position(times, positions, t_query):
    """Interpolate position at a given time"""
    if len(times) == 0:
        return np.zeros(3)
    if len(times) == 1:
        return positions[0]
    if t_query <= times[0]:
        return positions[0]
    if t_query >= times[-1]:
        return positions[-1]
    idx = np.searchsorted(times, t_query)
    if idx >= len(times):
        return positions[-1]
    t0, t1 = times[idx-1], times[idx]
    p0, p1 = positions[idx-1], positions[idx]
    alpha = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
    return p0 + alpha * (p1 - p0)


def analyze_mpc_compute_time_from_bag(bag_path: Path):
    """Extract MPC computation time statistics from rosbag"""
    if not bag_path.exists():
        return {'mpc_compute_time_avg': 0.0, 'mpc_compute_time_max': 0.0,
                'mpc_compute_time_std': 0.0, 'mpc_solve_count': 0}

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        return {'mpc_compute_time_avg': 0.0, 'mpc_compute_time_max': 0.0,
                'mpc_compute_time_std': 0.0, 'mpc_solve_count': 0}

    compute_times = []
    try:
        for topic, msg, t in bag.read_messages(topics=['/mpcNavigation/mpc_compute_time']):
            compute_times.append(msg.data)
    finally:
        bag.close()

    if not compute_times:
        return {'mpc_compute_time_avg': 0.0, 'mpc_compute_time_max': 0.0,
                'mpc_compute_time_std': 0.0, 'mpc_solve_count': 0}

    compute_times = np.array(compute_times)
    return {
        'mpc_compute_time_avg': float(np.mean(compute_times)),
        'mpc_compute_time_max': float(np.max(compute_times)),
        'mpc_compute_time_std': float(np.std(compute_times)),
        'mpc_solve_count': len(compute_times)
    }


def analyze_collision_from_bag(bag_path: Path, drone_bbox=(0.1, 0.1, 0.1)):
    """Analyze collisions from rosbag"""
    if not bag_path.exists():
        return {'collision_count': 0, 'min_distance': float('inf'),
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"ERROR opening bag: {e}")
        return {'collision_count': 0, 'min_distance': -1.0,
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    # Debug: print topics in bag
    bag_topics = bag.get_type_and_topic_info()[1].keys()
    print(f"topics={list(bag_topics)}", end=" ", flush=True)

    drone_trajectory = []
    obstacle_trajectories = {}
    obstacle_sizes = {}

    try:
        for topic, msg, t in bag.read_messages(topics=['/CERLAB/quadcopter/odom']):
            drone_trajectory.append((t.to_sec(), msg.pose.pose.position.x,
                                    msg.pose.pose.position.y, msg.pose.pose.position.z))

        # Read from /dynus/model_states (DYNUS obstacles), fallback to /gazebo/model_states
        for topic, msg, t in bag.read_messages(topics=['/dynus/model_states', '/gazebo/model_states']):
            for i, name in enumerate(msg.name):
                if 'obstacle' in name:
                    pos = msg.pose[i].position
                    if name not in obstacle_trajectories:
                        obstacle_trajectories[name] = []
                        obstacle_sizes[name] = parse_obstacle_size(name)
                    obstacle_trajectories[name].append((t.to_sec(), pos.x, pos.y, pos.z))
    finally:
        bag.close()

    if not drone_trajectory:
        print("WARNING: No drone trajectory in bag!")
        return {'collision_count': 0, 'min_distance': -1.0,
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    drone_times = np.array([t for t, x, y, z in drone_trajectory])
    drone_positions = np.array([[x, y, z] for t, x, y, z in drone_trajectory])

    obstacle_data = {}
    for obs_name, traj in obstacle_trajectories.items():
        if traj:
            times = np.array([t for t, x, y, z in traj])
            positions = np.array([[x, y, z] for t, x, y, z in traj])
            obstacle_data[obs_name] = {'times': times, 'positions': positions,
                                       'half_extents': obstacle_sizes[obs_name]}

    print(f"drone_pts={len(drone_trajectory)}, obstacles={len(obstacle_data)}", end=" ", flush=True)

    if not obstacle_data:
        print("WARNING: No obstacles found in bag!")
        return {'collision_count': 0, 'min_distance': -1.0,
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0,
                'collision_unique_obstacles': 0}

    collisions = 0
    min_distance = float('inf')
    max_penetration = 0.0
    collision_free_segments = 0
    collided_obstacles = set()
    drone_hx, drone_hy, drone_hz = drone_bbox

    for t, px, py, pz in zip(drone_times, drone_positions[:, 0],
                             drone_positions[:, 1], drone_positions[:, 2]):
        drone_bbox_obj = BoundingBox.from_center_and_half_extents(px, py, pz, drone_hx, drone_hy, drone_hz)
        segment_collision_free = True

        for obs_name, obs_data in obstacle_data.items():
            obs_pos = interpolate_position(obs_data['times'], obs_data['positions'], t)
            obs_half_extents = obs_data['half_extents']
            obs_bbox = BoundingBox.from_center_and_half_extents(
                obs_pos[0], obs_pos[1], obs_pos[2],
                obs_half_extents[0], obs_half_extents[1], obs_half_extents[2])

            distance = drone_bbox_obj.distance_to(obs_bbox)
            min_distance = min(min_distance, distance)

            if drone_bbox_obj.intersects(obs_bbox):
                segment_collision_free = False
                collisions += 1
                penetration = -distance if distance < 0 else 0.0
                max_penetration = max(max_penetration, penetration)
                collided_obstacles.add(obs_name)

        if segment_collision_free:
            collision_free_segments += 1

    return {
        'collision_count': collisions,
        'min_distance': min_distance if min_distance != float('inf') else -1.0,  # -1 = no data
        'collision_free_ratio': collision_free_segments / len(drone_positions) if drone_positions.size else 1.0,
        'collision_penetration_max': max_penetration,
        'collision_unique_obstacles': len(collided_obstacles)
    }


def postprocess_collisions(data_dir: Path, df: pd.DataFrame):
    """Post-process collisions from rosbags"""
    bags_dir = data_dir / "bags"

    if not bags_dir.exists() or not HAS_ROSBAG:
        if not HAS_ROSBAG:
            print("  Skipping collision post-processing (rosbag not available)")
        else:
            print("  Skipping collision post-processing (no bags directory)")
        return df

    print("\n" + "="*80)
    print("POST-PROCESSING COLLISIONS FROM ROSBAGS")
    print("="*80)

    for idx, row in df.iterrows():
        trial_id = row['trial_id']
        bag_path = bags_dir / f"trial_{trial_id}.bag"

        print(f"\nTrial {trial_id}: ", end="", flush=True)

        if not bag_path.exists():
            print("bag not found")
            continue

        result = analyze_collision_from_bag(bag_path)
        df.at[idx, 'collision_count'] = result['collision_count']
        df.at[idx, 'min_distance_to_obstacles'] = result['min_distance']
        df.at[idx, 'collision_free_ratio'] = result['collision_free_ratio']
        df.at[idx, 'collision_penetration_max'] = result['collision_penetration_max']
        df.at[idx, 'collision_unique_obstacles'] = result['collision_unique_obstacles']
        df.at[idx, 'collision'] = result['collision_count'] > 0

        # Extract MPC computation time
        mpc_result = analyze_mpc_compute_time_from_bag(bag_path)
        df.at[idx, 'mpc_compute_time_avg'] = mpc_result['mpc_compute_time_avg']
        df.at[idx, 'mpc_compute_time_max'] = mpc_result['mpc_compute_time_max']
        df.at[idx, 'mpc_compute_time_std'] = mpc_result['mpc_compute_time_std']
        df.at[idx, 'mpc_solve_count'] = mpc_result['mpc_solve_count']

        min_dist_str = f"{result['min_distance']:.3f}m" if result['min_distance'] >= 0 else "N/A"
        mpc_time_str = f"{mpc_result['mpc_compute_time_avg']*1000:.1f}ms" if mpc_result['mpc_solve_count'] > 0 else "N/A"
        print(f"-> collisions={result['collision_count']}, min_dist={min_dist_str}, mpc_time={mpc_time_str}")

    output_csv = data_dir / "benchmark_intent_mpc_postprocessed.csv"
    df.to_csv(output_csv, index=False)
    print(f"\n✓ Post-processed data saved: {output_csv}")

    return df


# ============================================================================
# STATISTICS AND ANALYSIS
# ============================================================================

def load_benchmark_data(data_dir: Path):
    """Load benchmark CSV"""
    csv_files = [f for f in data_dir.glob("benchmark_intent_mpc_*.csv")
                 if 'summary' not in f.name.lower() and 'postprocessed' not in f.name.lower()]

    if not csv_files:
        print(f"ERROR: No benchmark CSV found in {data_dir}")
        sys.exit(1)

    csv_file = sorted(csv_files, key=lambda f: f.stat().st_mtime)[-1]
    print(f"Loading: {csv_file.name}")
    return pd.read_csv(csv_file)


def compute_statistics(df: pd.DataFrame):
    """Compute comprehensive statistics

    IMPORTANT:
    - Success is defined as goal_reached AND collision_free (DYNUS methodology)
    - Performance, safety, and constraint violation metrics are computed
      ONLY from successful runs (which are already collision-free by definition)
    """
    stats = {'total_trials': len(df)}

    # Determine collision-free status
    if 'collision_count' in df.columns:
        collision_free_mask = df['collision_count'] == 0
    else:
        collision_free_mask = df['collision'] == False

    # SUCCESS = goal_reached AND collision_free (DYNUS definition)
    success_mask = (df['goal_reached'] == True) & collision_free_mask
    stats['success_rate'] = success_mask.mean() * 100  # R^{succ} in table

    # Additional breakdown stats (for debugging/analysis, not in table)
    stats['goal_reached_rate'] = df['goal_reached'].mean() * 100
    stats['collision_free_rate'] = collision_free_mask.mean() * 100
    stats['timeout_rate'] = df['timeout_reached'].mean() * 100
    stats['collision_rate'] = (~collision_free_mask).mean() * 100

    if 'collision_count' in df.columns:
        stats['collision_count_total'] = df['collision_count'].sum()

    # Successful runs (goal_reached AND collision_free)
    successful = df[success_mask]
    stats['n_successful'] = len(successful)

    # For clarity, valid_runs = successful (since success already implies collision-free)
    valid_runs = successful
    stats['n_valid_runs'] = len(valid_runs)

    print(f"\n  Success defined as: goal_reached AND collision_free")
    print(f"  Computing metrics from {len(valid_runs)} successful runs")
    print(f"    - Total trials: {len(df)}")
    print(f"    - Goal reached: {(df['goal_reached'] == True).sum()}")
    print(f"    - Collision-free: {collision_free_mask.sum()}")
    print(f"    - Successful (both): {len(valid_runs)} ({stats['success_rate']:.1f}%)")

    if len(valid_runs) == 0:
        print("  WARNING: No valid runs to compute performance metrics!")
        return stats

    # Performance metrics (from valid runs only)
    for metric in ['flight_travel_time', 'path_length', 'path_efficiency',
                   'jerk_rms', 'jerk_integral', 'avg_velocity', 'max_velocity',
                   'avg_acceleration', 'max_acceleration']:
        if metric in valid_runs.columns:
            values = valid_runs[metric].dropna()
            if len(values) > 0:
                stats[f'{metric}_mean'] = values.mean()
                stats[f'{metric}_std'] = values.std()

    # Constraint violations (from valid runs only)
    for viol_type in ['vel', 'acc', 'jerk']:
        col = f'{viol_type}_violation_count'
        if col in valid_runs.columns:
            viol_rate = (valid_runs[col] > 0).mean() * 100
            stats[f'{viol_type}_violation_rate'] = viol_rate

    # Safety metrics - min distance (from valid runs only)
    if 'min_distance_to_obstacles' in valid_runs.columns:
        values = valid_runs['min_distance_to_obstacles'].dropna()
        # Filter out inf values and -1 (no data marker)
        values = values[(values != float('inf')) & (values >= 0)]
        if len(values) > 0:
            stats['min_distance_to_obstacles_mean'] = values.mean()
            stats['min_distance_to_obstacles_std'] = values.std()
        else:
            stats['min_distance_to_obstacles_mean'] = None

    # MPC computation time (from valid runs only)
    if 'mpc_compute_time_avg' in valid_runs.columns:
        values = valid_runs['mpc_compute_time_avg'].dropna()
        values = values[values > 0]  # Filter out zeros (no data)
        if len(values) > 0:
            # Convert to milliseconds for reporting
            stats['mpc_compute_time_mean_ms'] = values.mean() * 1000
            stats['mpc_compute_time_std_ms'] = values.std() * 1000

    if 'mpc_compute_time_max' in valid_runs.columns:
        values = valid_runs['mpc_compute_time_max'].dropna()
        values = values[values > 0]
        if len(values) > 0:
            stats['mpc_compute_time_max_ms'] = values.max() * 1000

    if 'mpc_solve_count' in valid_runs.columns:
        values = valid_runs['mpc_solve_count'].dropna()
        if len(values) > 0:
            stats['mpc_solve_count_mean'] = values.mean()

    return stats


def print_statistics(stats: dict):
    """Print formatted statistics"""
    print("\n" + "="*80)
    print("BENCHMARK ANALYSIS RESULTS")
    print("="*80)
    print(f"\nOVERVIEW")
    print(f"  Total trials: {stats.get('total_trials', 0)}")
    print(f"  SUCCESS RATE (R^succ): {stats.get('success_rate', 0):.1f}%")
    print(f"    (Success = goal_reached AND collision_free)")
    print(f"  Successful runs: {stats.get('n_successful', 0)}")
    print(f"\n  Breakdown:")
    print(f"    - Goal reached: {stats.get('goal_reached_rate', 0):.1f}%")
    print(f"    - Collision-free: {stats.get('collision_free_rate', 0):.1f}%")
    print(f"    - Timeout: {stats.get('timeout_rate', 0):.1f}%")

    print(f"\nNOTE: Performance metrics computed from {stats.get('n_successful', 0)} successful runs only")

    if 'flight_travel_time_mean' in stats:
        print(f"\nPERFORMANCE (from valid runs)")
        print(f"  Travel time: {stats['flight_travel_time_mean']:.2f} ± {stats.get('flight_travel_time_std', 0):.2f} s")
        print(f"  Path length: {stats['path_length_mean']:.2f} ± {stats.get('path_length_std', 0):.2f} m")
        print(f"  Path efficiency: {stats.get('path_efficiency_mean', 0):.3f}")

    if 'mpc_compute_time_mean_ms' in stats:
        print(f"\nMPC COMPUTATION TIME (from valid runs)")
        print(f"  Average: {stats['mpc_compute_time_mean_ms']:.1f} ± {stats.get('mpc_compute_time_std_ms', 0):.1f} ms")
        if 'mpc_compute_time_max_ms' in stats:
            print(f"  Maximum: {stats['mpc_compute_time_max_ms']:.1f} ms")
        if 'mpc_solve_count_mean' in stats:
            print(f"  Avg solves per trial: {stats['mpc_solve_count_mean']:.0f}")

    if 'jerk_integral_mean' in stats:
        print(f"\nSMOOTHNESS (from valid runs)")
        print(f"  Jerk integral: {stats['jerk_integral_mean']:.2f}")

    print(f"\nSAFETY (from successful runs)")
    min_dist = stats.get('min_distance_to_obstacles_mean')
    if min_dist is not None:
        print(f"  Min distance to obstacles (d_min): {min_dist:.3f} m")
    else:
        print(f"  Min distance to obstacles: N/A (no obstacle data collected)")

    print(f"\nCONSTRAINT VIOLATIONS (from valid runs)")
    for viol in ['vel', 'acc', 'jerk']:
        if f'{viol}_violation_rate' in stats:
            print(f"  {viol.upper()} violation rate: {stats[f'{viol}_violation_rate']:.1f}%")


def update_dynus_latex_table(stats: dict, dynus_table_path: Path, algorithm_name: str = "I-MPC"):
    """Update DYNUS LaTeX table with I-MPC results

    Table format (9 columns):
    Algorithm | R^{succ} | T^{per}_{opt} | T_{trav} | L_{path} | S_{jerk} | d_{min} | rho_{vel} | rho_{acc} | rho_{jerk}

    Note: R^{succ} = success rate = goal_reached AND collision_free
    """
    if not dynus_table_path.exists():
        print(f"\nWarning: DYNUS table not found at {dynus_table_path}")
        return None

    # R^{succ} = success rate (goal_reached AND collision_free)
    success_rate = stats.get('success_rate', 0)

    # Use MPC computation time (in ms) for "per opt time" column
    per_opt_time = stats.get('mpc_compute_time_mean_ms', stats.get('avg_replanning_time_mean', 0))
    travel_time = stats.get('flight_travel_time_mean', 0)
    path_length = stats.get('path_length_mean', 0)
    jerk_integral = stats.get('jerk_integral_mean', 0)
    min_distance = stats.get('min_distance_to_obstacles_mean', 0)
    vel_viol = stats.get('vel_violation_rate', 0)
    acc_viol = stats.get('acc_violation_rate', 0)
    jerk_viol = stats.get('jerk_violation_rate', 0)

    min_dist_str = f"{min_distance:.3f}" if isinstance(min_distance, (int, float)) and min_distance is not None else "N/A"

    # 9 columns: Algorithm & R^succ & T^per_opt & T_trav & L_path & S_jerk & d_min & rho_vel & rho_acc & rho_jerk
    impc_row = (f"      {algorithm_name} & {success_rate:.1f} & "
                f"{per_opt_time:.1f} & {travel_time:.1f} & {path_length:.1f} & "
                f"{jerk_integral:.1f} & {min_dist_str} & {vel_viol:.1f} & "
                f"{acc_viol:.1f} & {jerk_viol:.1f} \\\\")

    with open(dynus_table_path, 'r') as f:
        lines = f.readlines()

    new_lines = []
    inserted = False
    for i, line in enumerate(lines):
        new_lines.append(line)
        if "DYNUS &" in line and not inserted:
            if i + 1 < len(lines) and ("I-MPC" in lines[i + 1] or "Intent-MPC" in lines[i + 1]):
                new_lines.append(impc_row + "\n")
                inserted = True
            else:
                new_lines.append(impc_row + "\n")
                inserted = True

    with open(dynus_table_path, 'w') as f:
        f.writelines(new_lines)

    print(f"\n✓ Updated DYNUS table: {dynus_table_path}")
    print(f"  Added/updated {algorithm_name} row")
    return dynus_table_path


def main():
    parser = argparse.ArgumentParser(
        description='Complete Intent-MPC benchmark analysis (post-processing + statistics + LaTeX)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--data-dir', type=str, required=True,
                        help='Path to benchmark data directory')
    parser.add_argument('--algorithm-name', type=str, default='I-MPC',
                        help='Algorithm name for LaTeX table (default: I-MPC)')
    parser.add_argument('--dynus-table-path', type=str,
                        default='/home/kkondo/paper_writing/DYNUS_v3/tables/dynamic_benchmark.tex',
                        help='Path to DYNUS LaTeX table')
    parser.add_argument('--skip-collision-postprocess', action='store_true',
                        help='Skip collision post-processing from rosbags')

    args = parser.parse_args()

    print("="*80)
    print("INTENT-MPC COMPLETE BENCHMARK ANALYSIS")
    print("="*80)

    data_dir = Path(args.data_dir)
    if not data_dir.exists():
        print(f"ERROR: Directory not found: {data_dir}")
        sys.exit(1)

    # Load data
    df = load_benchmark_data(data_dir)
    print(f"  Trials: {len(df)}\n")

    # Post-process collisions from rosbags
    if not args.skip_collision_postprocess:
        df = postprocess_collisions(data_dir, df)

    # Compute statistics
    print("\nComputing statistics...")
    stats = compute_statistics(df)

    # Print results
    print_statistics(stats)

    # Save summary CSV
    summary_csv = data_dir / "benchmark_summary.csv"
    pd.DataFrame([stats]).to_csv(summary_csv, index=False)
    print(f"\n✓ Summary saved: {summary_csv}")

    # Update DYNUS LaTeX table
    update_dynus_latex_table(stats, Path(args.dynus_table_path), args.algorithm_name)

    print("\n" + "="*80)
    print("ANALYSIS COMPLETE")
    print("="*80)
    print(f"\nGenerated files:")
    print(f"  1. {data_dir / 'benchmark_intent_mpc_postprocessed.csv'} (if rosbags processed)")
    print(f"  2. {summary_csv}")
    print(f"  3. {args.dynus_table_path} (updated)")
    print()


if __name__ == '__main__':
    main()
