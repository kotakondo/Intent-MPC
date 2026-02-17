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
HAS_ROSBAG = False
HAS_ROSBAGS = False
try:
    import rosbag
    HAS_ROSBAG = True
except ImportError:
    pass

# Also try rosbags (ROS 2 compatible reader for ROS 1 bags)
try:
    from rosbags.rosbag1 import Reader as RosbagReader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    HAS_ROSBAGS = True
except ImportError:
    pass

if not HAS_ROSBAG and not HAS_ROSBAGS:
    print("Warning: Neither rosbag nor rosbags available. Will skip bag-based post-processing.")
    print("Install with: pip install rosbags")


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


def recompute_violations_from_bag(bag_path, vel_limit=5.0, acc_limit=20.0,
                                   jerk_limit=100.0, tolerance=1e-3):
    """Recompute constraint violations from a ROS1 bag in post-processing.

    Reads /autonomous_flight/target_state (Target.msg with position, velocity,
    acceleration fields) and counts per-timestep violations using Linf norm:
    a violation at a single timestep occurs when ANY axis component exceeds the
    limit + tolerance.

    Jerk is computed via finite differences on acceleration.

    Returns dict with:
        vel_violation_count, vel_total,
        acc_violation_count, acc_total,
        jerk_violation_count, jerk_total
    """
    empty = {
        'vel_violation_count': 0, 'vel_total': 0,
        'acc_violation_count': 0, 'acc_total': 0,
        'jerk_violation_count': 0, 'jerk_total': 0,
    }

    bag_path = Path(bag_path)
    if not bag_path.exists() or not HAS_ROSBAG:
        return empty

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"  ERROR opening bag for violations: {e}")
        return empty

    # Read commanded trajectory from /autonomous_flight/target_state
    times = []
    velocities = []
    accelerations = []

    try:
        for topic, msg, t in bag.read_messages(
                topics=['/autonomous_flight/target_state']):
            stamp = t.to_sec()
            vel = (msg.velocity.x, msg.velocity.y, msg.velocity.z)
            acc = (msg.acceleration.x, msg.acceleration.y, msg.acceleration.z)
            times.append(stamp)
            velocities.append(vel)
            accelerations.append(acc)
    finally:
        bag.close()

    if not times:
        return empty

    times = np.array(times)
    velocities = np.array(velocities)       # (N, 3)
    accelerations = np.array(accelerations)  # (N, 3)

    n = len(times)

    # --- Velocity violations (Linf: any axis exceeds limit) ---
    vel_violation_count = 0
    for i in range(n):
        if (abs(velocities[i, 0]) > vel_limit + tolerance or
                abs(velocities[i, 1]) > vel_limit + tolerance or
                abs(velocities[i, 2]) > vel_limit + tolerance):
            vel_violation_count += 1

    # --- Acceleration violations (Linf: any axis exceeds limit) ---
    acc_violation_count = 0
    for i in range(n):
        if (abs(accelerations[i, 0]) > acc_limit + tolerance or
                abs(accelerations[i, 1]) > acc_limit + tolerance or
                abs(accelerations[i, 2]) > acc_limit + tolerance):
            acc_violation_count += 1

    # --- Jerk violations via finite differences (Linf) ---
    jerk_violation_count = 0
    jerk_total = 0
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        if dt > 0.001:
            jx = (accelerations[i, 0] - accelerations[i - 1, 0]) / dt
            jy = (accelerations[i, 1] - accelerations[i - 1, 1]) / dt
            jz = (accelerations[i, 2] - accelerations[i - 1, 2]) / dt
            jerk_total += 1
            if (abs(jx) > jerk_limit + tolerance or
                    abs(jy) > jerk_limit + tolerance or
                    abs(jz) > jerk_limit + tolerance):
                jerk_violation_count += 1

    return {
        'vel_violation_count': vel_violation_count,
        'vel_total': n,
        'acc_violation_count': acc_violation_count,
        'acc_total': n,
        'jerk_violation_count': jerk_violation_count,
        'jerk_total': jerk_total,
    }


def recompute_violations_from_bag_rosbags(bag_path, vel_limit=5.0, acc_limit=20.0,
                                           jerk_limit=100.0, tolerance=1e-3):
    """Recompute constraint violations using rosbags library (ROS 2 compatible).

    Same methodology as recompute_violations_from_bag() but uses rosbags instead
    of rosbag, avoiding ROS 1/2 conflicts.
    """
    empty = {
        'vel_violation_count': 0, 'vel_total': 0,
        'acc_violation_count': 0, 'acc_total': 0,
        'jerk_violation_count': 0, 'jerk_total': 0,
    }

    bag_path = Path(bag_path)
    if not bag_path.exists() or not HAS_ROSBAGS:
        return empty

    # Register custom Target message type
    typestore = get_typestore(Stores.ROS1_NOETIC)
    target_msgdef = (
        'std_msgs/Header header\n'
        'uint8 type_mask\n'
        'uint8 IGNORE_ACC = 1\n'
        'uint8 IGNORE_ACC_VEL = 2\n'
        'geometry_msgs/Vector3 position\n'
        'geometry_msgs/Vector3 velocity\n'
        'geometry_msgs/Vector3 acceleration\n'
        'float32 yaw\n'
    )
    try:
        add_types = get_types_from_msg(target_msgdef, 'tracking_controller/msg/Target')
        typestore.register(add_types)
    except Exception:
        pass  # Already registered

    times = []
    velocities = []
    accelerations = []

    try:
        with RosbagReader(str(bag_path)) as reader:
            conns = [c for c in reader.connections
                     if c.topic == '/autonomous_flight/target_state']
            for conn, timestamp, rawdata in reader.messages(connections=conns):
                msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
                times.append(timestamp * 1e-9)  # nanoseconds to seconds
                velocities.append((msg.velocity.x, msg.velocity.y, msg.velocity.z))
                accelerations.append((msg.acceleration.x, msg.acceleration.y, msg.acceleration.z))
    except Exception as e:
        print(f"  ERROR reading bag with rosbags: {e}")
        return empty

    if not times:
        return empty

    times = np.array(times)
    velocities = np.array(velocities)
    accelerations = np.array(accelerations)
    n = len(times)

    # Velocity violations (Linf: any axis exceeds limit)
    vel_viol = np.any(np.abs(velocities) > vel_limit + tolerance, axis=1)
    vel_violation_count = int(np.sum(vel_viol))

    # Acceleration violations (Linf: any axis exceeds limit)
    acc_viol = np.any(np.abs(accelerations) > acc_limit + tolerance, axis=1)
    acc_violation_count = int(np.sum(acc_viol))

    # Jerk violations via finite differences (Linf)
    jerk_violation_count = 0
    jerk_total = 0
    dt = np.diff(times)
    valid = dt > 0.001
    if np.any(valid):
        d_acc = np.diff(accelerations, axis=0)
        jerks = d_acc[valid] / dt[valid, np.newaxis]
        jerk_total = len(jerks)
        jerk_viol = np.any(np.abs(jerks) > jerk_limit + tolerance, axis=1)
        jerk_violation_count = int(np.sum(jerk_viol))

    return {
        'vel_violation_count': vel_violation_count,
        'vel_total': n,
        'acc_violation_count': acc_violation_count,
        'acc_total': n,
        'jerk_violation_count': jerk_violation_count,
        'jerk_total': jerk_total,
    }


def analyze_collision_from_bag(bag_path: Path, drone_bbox=(0.0, 0.0, 0.0)):
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
        # Use index as key since all obstacles of the same type share the same name
        for topic, msg, t in bag.read_messages(topics=['/dynus/model_states', '/gazebo/model_states']):
            for i, name in enumerate(msg.name):
                if 'obstacle' in name:
                    pos = msg.pose[i].position
                    obs_key = i
                    if obs_key not in obstacle_trajectories:
                        obstacle_trajectories[obs_key] = []
                        obstacle_sizes[obs_key] = parse_obstacle_size(name)
                    obstacle_trajectories[obs_key].append((t.to_sec(), pos.x, pos.y, pos.z))
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

            # Euclidean distance from drone center to closest point on obstacle
            # AABB surface, consistent with DYNUS/FAPP/Ego-Swarm analysis scripts.
            # Does NOT account for drone size — measures from drone center point.
            hx, hy, hz = obs_half_extents
            dx = max(0.0, abs(px - obs_pos[0]) - hx)
            dy = max(0.0, abs(py - obs_pos[1]) - hy)
            dz = max(0.0, abs(pz - obs_pos[2]) - hz)
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            min_distance = min(min_distance, distance)

            if drone_bbox_obj.intersects(obs_bbox):
                segment_collision_free = False
                collisions += 1
                max_penetration = 0.0  # Not meaningful for per-axis distance
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

        # Recompute constraint violations from bag (Linf norm, per-timestep)
        viol_result = recompute_violations_from_bag(bag_path)
        df.at[idx, 'bag_vel_violation_count'] = viol_result['vel_violation_count']
        df.at[idx, 'bag_vel_total'] = viol_result['vel_total']
        df.at[idx, 'bag_acc_violation_count'] = viol_result['acc_violation_count']
        df.at[idx, 'bag_acc_total'] = viol_result['acc_total']
        df.at[idx, 'bag_jerk_violation_count'] = viol_result['jerk_violation_count']
        df.at[idx, 'bag_jerk_total'] = viol_result['jerk_total']

        min_dist_str = f"{result['min_distance']:.3f}m" if result['min_distance'] >= 0 else "N/A"
        mpc_time_str = f"{mpc_result['mpc_compute_time_avg']*1000:.1f}ms" if mpc_result['mpc_solve_count'] > 0 else "N/A"
        viol_str = (f"vel={viol_result['vel_violation_count']}/{viol_result['vel_total']}, "
                    f"acc={viol_result['acc_violation_count']}/{viol_result['acc_total']}, "
                    f"jerk={viol_result['jerk_violation_count']}/{viol_result['jerk_total']}")
        print(f"-> collisions={result['collision_count']}, min_dist={min_dist_str}, mpc_time={mpc_time_str}, violations=[{viol_str}]")

    output_csv = data_dir / "benchmark_intent_mpc_postprocessed.csv"
    df.to_csv(output_csv, index=False)
    print(f"\n✓ Post-processed data saved: {output_csv}")

    return df


def recompute_violations_from_bags(data_dir: Path, df: pd.DataFrame):
    """Recompute per-timestep Linf violations from bags when CSV lacks those columns.

    Uses the rosbags library (ROS 2 compatible) to read ROS 1 bags.
    Adds vel_violation_count, vel_total_samples, acc_violation_count, etc.
    to the DataFrame.
    """
    if not HAS_ROSBAGS:
        return df

    # Check if per-timestep columns already exist with valid data
    if ('vel_violation_count' in df.columns and 'vel_total_samples' in df.columns
            and df['vel_total_samples'].fillna(0).sum() > 0):
        return df  # Already have correct data

    bags_dir = data_dir / "bags"
    if not bags_dir.exists():
        return df

    print("\n  Recomputing per-timestep Linf violations from bags...")

    for idx, row in df.iterrows():
        trial_id = row['trial_id']
        bag_path = bags_dir / f"trial_{trial_id}.bag"

        if not bag_path.exists():
            continue

        # Read vel/acc limits from CSV if available
        vel_limit = row.get('vel_limit', 5.0)
        acc_limit = row.get('acc_limit', 20.0)
        jerk_limit = row.get('jerk_limit', 100.0)
        if pd.isna(vel_limit):
            vel_limit = 5.0
        if pd.isna(acc_limit):
            acc_limit = 20.0
        if pd.isna(jerk_limit):
            jerk_limit = 100.0

        result = recompute_violations_from_bag_rosbags(
            bag_path, vel_limit=vel_limit, acc_limit=acc_limit,
            jerk_limit=jerk_limit)

        df.at[idx, 'vel_violation_count'] = result['vel_violation_count']
        df.at[idx, 'vel_total_samples'] = result['vel_total']
        df.at[idx, 'acc_violation_count'] = result['acc_violation_count']
        df.at[idx, 'acc_total_samples'] = result['acc_total']
        df.at[idx, 'jerk_violation_count'] = result['jerk_violation_count']
        df.at[idx, 'jerk_total_samples'] = result['jerk_total']

        if result['vel_total'] > 0:
            vel_rate = result['vel_violation_count'] / result['vel_total'] * 100
            acc_rate = result['acc_violation_count'] / result['acc_total'] * 100
            jerk_rate = (result['jerk_violation_count'] / result['jerk_total'] * 100
                         if result['jerk_total'] > 0 else 0)
            print(f"    Trial {trial_id}: vel={vel_rate:.1f}% ({result['vel_violation_count']}/{result['vel_total']}), "
                  f"acc={acc_rate:.1f}% ({result['acc_violation_count']}/{result['acc_total']}), "
                  f"jerk={jerk_rate:.1f}% ({result['jerk_violation_count']}/{result['jerk_total']})")

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
    # DYNUS methodology: violation rate = violating_timesteps / total_timesteps × 100%
    # where a timestep is a violation if ANY axis exceeds the limit (Linf).
    # Priority: 1) CSV per-timestep Linf columns, 2) bag-recomputed, 3) per-axis fallback
    for viol_type in ['vel', 'acc', 'jerk']:
        # Option 1: Per-timestep Linf columns from runner (preferred)
        count_col = f'{viol_type}_violation_count'
        total_col = f'{viol_type}_total_samples'
        if count_col in valid_runs.columns and total_col in valid_runs.columns:
            total_violations = valid_runs[count_col].fillna(0).sum()
            total_samples = valid_runs[total_col].fillna(0).sum()
            if total_samples > 0:
                viol_rate = total_violations / total_samples * 100.0
            else:
                viol_rate = 0.0
            stats[f'{viol_type}_violation_rate'] = viol_rate
            continue

        # Option 2: Bag-recomputed Linf violations (post-processed from rosbags)
        bag_count_col = f'bag_{viol_type}_violation_count'
        bag_total_col = f'bag_{viol_type}_total'
        if bag_count_col in valid_runs.columns and bag_total_col in valid_runs.columns:
            total_violations = valid_runs[bag_count_col].fillna(0).sum()
            total_samples = valid_runs[bag_total_col].fillna(0).sum()
            if total_samples > 0:
                viol_rate = total_violations / total_samples * 100.0
            else:
                viol_rate = 0.0
            stats[f'{viol_type}_violation_rate'] = viol_rate
            continue

        # Option 3: Per-axis columns fallback (old data without per-timestep counts).
        # Without total_samples, we cannot compute the exact timestep-based rate.
        # Fall back to: fraction of runs that had at least one violation.
        x_col = f'{viol_type}_violation_count_x'
        y_col = f'{viol_type}_violation_count_y'
        z_col = f'{viol_type}_violation_count_z'
        if x_col in valid_runs.columns:
            total_viols_per_run = (valid_runs[x_col].fillna(0)
                                   + valid_runs[y_col].fillna(0)
                                   + valid_runs[z_col].fillna(0))
            viol_rate = (total_viols_per_run > 0).mean() * 100
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


def _format_wa_label(acceleration_weight):
    """Format acceleration_weight as $w_a$=\\num{eN} for powers of 10"""
    if acceleration_weight is not None and acceleration_weight > 0:
        try:
            exp = round(math.log10(acceleration_weight))
            if acceleration_weight == 10 ** exp:
                return f'$w_a$=\\num{{e{exp}}}'
        except (ValueError, OverflowError):
            pass
        return f'$w_a$={acceleration_weight}'
    return ''


def _format_vmax_label(max_vel):
    """Format max_vel as $v_{max}$=X.X for LaTeX"""
    if max_vel is not None and max_vel > 0:
        if max_vel == int(max_vel):
            return f'$v_{{\\max}}$={int(max_vel)}'
        return f'$v_{{\\max}}$={max_vel}'
    return ''


def parse_config_from_dirname(dirname: str):
    """Parse max_vel and max_acc from directory name like 'max_vel_3_acc_3'."""
    import re
    match = re.match(r'max_vel_(\d+(?:\.\d+)?)_acc_(\d+(?:\.\d+)?)', dirname)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None, None


def _format_impc_metric_values(stats):
    """Format the 9 metric values for a LaTeX table row.

    Columns: R_succ, T_per_opt, T_trav, L_path, S_jerk, d_min, rho_vel, rho_acc, rho_jerk

    When there are no successful runs, outputs {-} for all metrics except success rate.
    """
    success_rate = stats.get('success_rate', 0)
    n_valid = stats.get('n_valid_runs', 0)

    if n_valid == 0:
        dash = "{-}"
        return f"{success_rate:.1f} & {dash} & {dash} & {dash} & {dash} & {dash} & {dash} & {dash} & {dash}"

    per_opt_time = stats.get('mpc_compute_time_mean_ms', stats.get('avg_replanning_time_mean', 0))
    travel_time = stats.get('flight_travel_time_mean', 0)
    path_length = stats.get('path_length_mean', 0)
    jerk_integral = stats.get('jerk_integral_mean', 0)
    min_distance = stats.get('min_distance_to_obstacles_mean', 0)
    if min_distance is None:
        min_distance = 0
    vel_viol = stats.get('vel_violation_rate', 0)
    acc_viol = stats.get('acc_violation_rate', 0)
    jerk_str = "{-}"  # I-MPC has no jerk constraints

    return (f"{success_rate:.1f} & {per_opt_time:.1f} & {travel_time:.1f} & "
            f"{path_length:.1f} & {jerk_integral:.1f} & {min_distance:.3f} & "
            f"{vel_viol:.1f} & {acc_viol:.1f} & {jerk_str}")


def update_dynus_latex_table(stats: dict, dynus_table_path: Path, algorithm_name: str = "I-MPC"):
    """Update DYNUS LaTeX table with a single I-MPC row (for single-dir analysis).

    Uses the same in-place replacement as the grouped function: finds the I-MPC
    row in the matching case group and replaces its metrics.
    """
    # Delegate to the grouped function with a single-element list
    return update_dynus_latex_table_grouped([stats], dynus_table_path, group_name=algorithm_name)


def update_dynus_latex_table_grouped(all_stats: list, dynus_table_path: Path,
                                     group_name: str = "I-MPC"):
    """Update I-MPC rows in the DYNUS LaTeX table (in-place replacement).

    The table has case groups (Easy/Medium/Hard), each with algorithm rows:
      \\multirow{4}{*}{Easy} & DYNUS  & metrics... \\\\
                             & I-MPC  & metrics... \\\\
                             & FAPP   & metrics... \\\\
                             & EGO-v2 & metrics... \\\\

    This function finds the I-MPC row in each case group and replaces its
    metrics.  If no I-MPC row exists for a case, one is inserted after the
    first algorithm row of that group.
    """
    if not dynus_table_path.exists():
        print(f"\nWarning: DYNUS table not found at {dynus_table_path}")
        return None

    # Build difficulty -> stats map
    stats_by_difficulty = {}
    for s in all_stats:
        diff = s.get('difficulty')
        if diff:
            stats_by_difficulty[diff] = s

    if not stats_by_difficulty:
        print("Warning: No difficulty-level stats to write")
        return None

    with open(dynus_table_path, 'r') as f:
        lines = f.readlines()

    current_case = None
    result = []
    cases_updated = set()

    for line in lines:
        stripped = line.strip()

        # Detect case group start from \multirow{...}{Easy/Medium/Hard}
        for case_name, case_key in [('Easy', 'easy'), ('Medium', 'medium'), ('Hard', 'hard')]:
            if f'{{{case_name}}}' in stripped and '\\multirow' in stripped:
                current_case = case_key
                break

        # Replace existing I-MPC row with actual metrics
        if 'I-MPC' in stripped and '\\\\' in stripped:
            if current_case and current_case in stats_by_difficulty:
                stats = stats_by_difficulty[current_case]
                metrics = _format_impc_metric_values(stats)
                leading_ws = line[:len(line) - len(line.lstrip())]
                result.append(f"{leading_ws}& I-MPC & {metrics} \\\\\n")
                cases_updated.add(current_case)
            else:
                result.append(line)  # Keep placeholder as-is
        else:
            # At group boundary, insert I-MPC if missing for this case
            if ('\\midrule' in stripped or '\\bottomrule' in stripped) and current_case:
                if current_case in stats_by_difficulty and current_case not in cases_updated:
                    stats = stats_by_difficulty[current_case]
                    metrics = _format_impc_metric_values(stats)
                    result.append(f"                            & I-MPC & {metrics} \\\\\n")
                    cases_updated.add(current_case)
                if '\\midrule' in stripped:
                    current_case = None  # Reset at group boundary
            result.append(line)

    with open(dynus_table_path, 'w') as f:
        f.writelines(result)

    print(f"\nUpdated DYNUS table: {dynus_table_path}")
    for case in sorted(cases_updated):
        print(f"  {'Replaced' if case in cases_updated else 'Inserted'} I-MPC row for {case.capitalize()}")
    missing = set(stats_by_difficulty.keys()) - cases_updated
    for case in sorted(missing):
        print(f"  WARNING: No {case.capitalize()} case group found in table")
    return dynus_table_path


def update_dynus_latex_table_multi_config(configs: list, dynus_table_path: Path):
    """Update DYNUS LaTeX table with multiple I-MPC configurations.

    The table uses a 2-column algorithm format (12 columns total):
      & I-MPC & \\multicolumn{1}{l}{$v_{\\max}$=3} & metrics... \\\\
      & \\multicolumn{2}{c}{FAPP} & metrics... \\\\

    Args:
        configs: list of (display_label, vmax_label_or_None, {difficulty: stats}) tuples.
            vmax_label is e.g. "$v_{\\max}$=3" or None for a single unnamed config.
        dynus_table_path: Path to the LaTeX table file.

    For each case group (Easy/Medium/Hard), this function:
    1. Removes all existing I-MPC rows
    2. Inserts new rows (one per config) at the same position
    3. Updates the \\multirow{N} count to reflect the new row count
    """
    import re

    if not dynus_table_path.exists():
        print(f"\nWarning: DYNUS table not found at {dynus_table_path}")
        return None

    with open(dynus_table_path, 'r') as f:
        lines = f.readlines()

    current_case = None
    result = []
    impc_inserted = {}  # case_key -> True if we already inserted new rows

    for line in lines:
        stripped = line.strip()

        # Detect case group from \multirow{N}{*}{Easy/Medium/Hard}
        for case_name, case_key in [('Easy', 'easy'), ('Medium', 'medium'), ('Hard', 'hard')]:
            if f'{{{case_name}}}' in stripped and '\\multirow' in stripped:
                current_case = case_key
                # Update \multirow{N} count: count non-I-MPC algorithms + new I-MPC configs
                n_impc = sum(1 for _, _, sbd in configs if case_key in sbd)
                # 3 = DYNUS + FAPP + EGO-v2 (existing non-I-MPC algorithms)
                new_count = 3 + n_impc
                line = re.sub(r'(\\multirow)\{(\d+)\}',
                              r'\1{' + str(new_count) + '}', line)
                break

        # Handle existing I-MPC rows: remove them and insert new ones at first occurrence
        if 'I-MPC' in stripped and '\\\\' in stripped:
            if current_case and current_case not in impc_inserted:
                # Insert all config rows in place of the first I-MPC row
                leading_ws = line[:len(line) - len(line.lstrip())]
                for display_label, vmax_label, stats_by_diff in configs:
                    if current_case in stats_by_diff:
                        stats = stats_by_diff[current_case]
                        metrics = _format_impc_metric_values(stats)
                        if vmax_label:
                            # 2-column format: I-MPC | left-aligned vmax
                            result.append(f"{leading_ws}& I-MPC & \\multicolumn{{1}}{{l}}{{{vmax_label}}} & {metrics} \\\\\n")
                        else:
                            # Single config: span both algorithm columns
                            result.append(f"{leading_ws}& \\multicolumn{{2}}{{c}}{{I-MPC}} & {metrics} \\\\\n")
                impc_inserted[current_case] = True
            # Skip the original I-MPC line (whether first or subsequent)
            continue

        # Reset case tracking at group boundaries
        if '\\midrule' in stripped:
            current_case = None

        result.append(line)

    with open(dynus_table_path, 'w') as f:
        f.writelines(result)

    print(f"\nUpdated DYNUS table with multi-config I-MPC: {dynus_table_path}")
    for case_key in ['easy', 'medium', 'hard']:
        if case_key in impc_inserted:
            labels = [dl for dl, _, sbd in configs if case_key in sbd]
            print(f"  {case_key.capitalize()}: {len(labels)} I-MPC rows ({', '.join(labels)})")
    return dynus_table_path


def analyze_multi_config(config_dirs: list, dynus_table_path: Path,
                         skip_collision_postprocess: bool):
    """Analyze multiple configuration directories, each containing easy/medium/hard benchmarks.

    Each config_dir is expected to be named like 'max_vel_3_acc_3' and contain
    subdirectories like 'easy_benchmark_TIMESTAMP/', 'medium_benchmark_TIMESTAMP/', etc.
    """
    all_configs = []  # List of (label, {difficulty: stats})

    for config_dir in config_dirs:
        config_dir = Path(config_dir)
        max_vel, max_acc = parse_config_from_dirname(config_dir.name)

        if max_vel is not None:
            label = f"I-MPC {_format_vmax_label(max_vel)}"
        else:
            label = f"I-MPC ({config_dir.name})"

        print(f"\n{'#' * 80}")
        print(f"# CONFIG: {config_dir.name} -> {label}")
        print(f"{'#' * 80}")

        # Find difficulty subdirectories
        stats_by_diff = {}
        for diff in ['easy', 'medium', 'hard']:
            candidates = sorted(config_dir.glob(f"{diff}_benchmark_*"))
            if candidates:
                benchmark_dir = candidates[-1]  # Latest
                stats = analyze_single(benchmark_dir, 'I-MPC', dynus_table_path,
                                       skip_collision_postprocess,
                                       skip_table_update=True)
                if stats:
                    stats['difficulty'] = diff
                    stats['max_vel'] = max_vel
                    stats['max_acc'] = max_acc
                    stats_by_diff[diff] = stats

        if stats_by_diff:
            vmax_label = _format_vmax_label(max_vel) if max_vel is not None else None
            all_configs.append((label, vmax_label, stats_by_diff))

    if not all_configs:
        print("\nNo results to summarize.")
        return

    # If only one config, drop the vmax label (span both algorithm columns)
    if len(all_configs) == 1:
        all_configs = [("I-MPC", None, all_configs[0][2])]

    # Update DYNUS LaTeX table with all configs
    update_dynus_latex_table_multi_config(all_configs, Path(dynus_table_path))

    # Print combined summary
    print(f"\n{'=' * 80}")
    print("MULTI-CONFIG SWEEP SUMMARY")
    print("=" * 80)
    header = f"{'Config':<25} {'Case':<8} {'R_succ':>7} {'T_opt':>6} {'T_trav':>7} {'L_path':>7} {'S_jerk':>7} {'d_min':>7} {'rho_v':>6}"
    print(header)
    print("-" * len(header))
    for label, vmax_label, stats_by_diff in all_configs:
        for diff in ['easy', 'medium', 'hard']:
            if diff in stats_by_diff:
                s = stats_by_diff[diff]
                r_succ = s.get('success_rate', 0)
                t_opt = s.get('mpc_compute_time_mean_ms', 0)
                t_trav = s.get('flight_travel_time_mean', 0)
                l_path = s.get('path_length_mean', 0)
                s_jerk = s.get('jerk_integral_mean', 0)
                d_min_val = s.get('min_distance_to_obstacles_mean', None)
                d_min = f"{d_min_val:.3f}" if d_min_val is not None else "N/A"
                rho_v = s.get('vel_violation_rate', 0)
                print(f"{label:<25} {diff.capitalize():<8} {r_succ:>6.1f}% {t_opt:>6.1f} {t_trav:>7.1f} {l_path:>7.1f} {s_jerk:>7.1f} {d_min:>7} {rho_v:>5.1f}%")


def detect_acceleration_weight(data_dir: Path, df):
    """Auto-detect acceleration_weight from CSV column or directory name"""
    if 'acceleration_weight' in df.columns:
        wt = df['acceleration_weight'].dropna().unique()
        if len(wt) == 1:
            return float(wt[0])

    # Try to parse from directory name: benchmark_wa<value>_<timestamp>
    import re
    match = re.search(r'_wa([\d.]+)', data_dir.name)
    if match:
        return float(match.group(1))

    return None


def detect_max_vel(data_dir: Path, df):
    """Auto-detect max_vel from CSV column or directory name"""
    if 'max_vel' in df.columns:
        mv = df['max_vel'].dropna().unique()
        if len(mv) == 1:
            return float(mv[0])

    # Try to parse from directory name: benchmark_vmax<value>_<timestamp>
    import re
    match = re.search(r'_vmax([\d.]+)', data_dir.name)
    if match:
        return float(match.group(1))

    return None


def detect_difficulty(data_dir: Path, df):
    """Auto-detect difficulty from CSV column or directory name.

    Returns one of: 'easy', 'medium', 'hard', or None.
    """
    # Check CSV column first
    if 'difficulty' in df.columns:
        diff = df['difficulty'].dropna().unique()
        if len(diff) == 1 and diff[0] in ('easy', 'medium', 'hard'):
            return str(diff[0])

    # Try to parse from directory name: {difficulty}_benchmark_{timestamp}
    import re
    match = re.match(r'(easy|medium|hard)_benchmark_', data_dir.name)
    if match:
        return match.group(1)

    # Infer from num_obstacles
    obs_to_diff = {50: 'easy', 100: 'medium', 200: 'hard'}
    if 'num_obstacles' in df.columns:
        obs = df['num_obstacles'].dropna().unique()
        if len(obs) == 1:
            return obs_to_diff.get(int(obs[0]))

    return None


def format_weight_latex(value: float) -> str:
    """Format weight value as \\num{eN} for LaTeX (e.g. 10.0 -> \\num{e1}, 1000.0 -> \\num{e3})"""
    import math
    exp = math.log10(value)
    if exp == int(exp):
        return f'\\num{{e{int(exp)}}}'
    return str(value)


def analyze_single(data_dir: Path, algorithm_name: str, dynus_table_path: Path,
                   skip_collision_postprocess: bool, acceleration_weight: float = None,
                   skip_table_update: bool = False):
    """Run analysis on a single benchmark data directory. Returns stats dict."""
    print("=" * 80)
    print(f"INTENT-MPC BENCHMARK ANALYSIS: {data_dir.name}")
    print("=" * 80)

    if not data_dir.exists():
        print(f"ERROR: Directory not found: {data_dir}")
        return None

    df = load_benchmark_data(data_dir)
    print(f"  Trials: {len(df)}\n")

    # Auto-detect acceleration_weight if not explicitly provided
    if acceleration_weight is None:
        acceleration_weight = detect_acceleration_weight(data_dir, df)

    # Auto-detect difficulty
    difficulty = detect_difficulty(data_dir, df)

    # Auto-name algorithm (keep generic for grouped table)
    if difficulty is not None:
        print(f"  Auto-detected difficulty={difficulty}")
    elif acceleration_weight is not None and algorithm_name == 'I-MPC':
        algorithm_name = f'I-MPC, $w_a={format_weight_latex(acceleration_weight)}$'
        print(f"  Auto-detected acceleration_weight={acceleration_weight}, algorithm_name={algorithm_name}")

    if not skip_collision_postprocess:
        df = postprocess_collisions(data_dir, df)

    # Recompute per-timestep Linf violations from bags if CSV lacks those columns
    df = recompute_violations_from_bags(data_dir, df)

    print("\nComputing statistics...")
    stats = compute_statistics(df)
    stats['algorithm_name'] = algorithm_name
    stats['acceleration_weight'] = acceleration_weight
    stats['difficulty'] = difficulty

    print_statistics(stats)

    # Save summary CSV
    summary_csv = data_dir / "benchmark_summary.csv"
    try:
        pd.DataFrame([stats]).to_csv(summary_csv, index=False)
        print(f"\n✓ Summary saved: {summary_csv}")
    except PermissionError:
        print(f"\n  (Skipped writing {summary_csv} - permission denied)")

    # Update DYNUS LaTeX table (skip when called from analyze_multi)
    if not skip_table_update:
        update_dynus_latex_table(stats, dynus_table_path, algorithm_name)

    return stats


def analyze_multi(data_dirs: list, algorithm_name: str, dynus_table_path: Path,
                  skip_collision_postprocess: bool):
    """Analyze multiple benchmark directories and print a combined summary table."""
    all_stats = []
    for data_dir in data_dirs:
        data_dir = Path(data_dir)
        stats = analyze_single(data_dir, algorithm_name, dynus_table_path,
                               skip_collision_postprocess,
                               skip_table_update=True)
        if stats is not None:
            all_stats.append(stats)
        print()

    if not all_stats:
        print("No results to summarize.")
        return

    # Update DYNUS LaTeX table with grouped multirow format
    update_dynus_latex_table_grouped(all_stats, Path(dynus_table_path), group_name='I-MPC')

    # Print combined summary table
    print("\n" + "=" * 80)
    print("SWEEP SUMMARY")
    print("=" * 80)
    header = f"{'Case':<10} {'R_succ':>7} {'T_trav':>7} {'L_path':>7} {'S_jerk':>7} {'d_min':>7} {'rho_v':>6} {'rho_a':>6}"
    print(header)
    print("-" * len(header))
    for s in all_stats:
        diff = s.get('difficulty', '')
        case_label = diff.capitalize() if diff else s.get('algorithm_name', 'I-MPC')
        r_succ = s.get('success_rate', 0)
        t_trav = s.get('flight_travel_time_mean', 0)
        l_path = s.get('path_length_mean', 0)
        s_jerk = s.get('jerk_integral_mean', 0)
        d_min_val = s.get('min_distance_to_obstacles_mean', None)
        d_min = f"{d_min_val:.3f}" if d_min_val is not None else "N/A"
        rho_v = s.get('vel_violation_rate', 0)
        rho_a = s.get('acc_violation_rate', 0)
        print(f"{case_label:<10} {r_succ:>6.1f}% {t_trav:>7.1f} {l_path:>7.1f} {s_jerk:>7.1f} {d_min:>7} {rho_v:>5.1f}% {rho_a:>5.1f}%")


def main():
    parser = argparse.ArgumentParser(
        description='Complete Intent-MPC benchmark analysis (post-processing + statistics + LaTeX)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--data-dir', type=str,
                        help='Path to single benchmark data directory')
    parser.add_argument('--multi-dir', type=str, nargs='+',
                        help='Multiple benchmark data directories for sweep analysis')
    parser.add_argument('--config-dirs', type=str, nargs='+',
                        help='Top-level config directories (e.g., data/max_vel_3_acc_3 data/max_vel_5_acc_20). '
                             'Each should contain {easy,medium,hard}_benchmark_* subdirs.')
    parser.add_argument('--acceleration-weight', type=float, default=None,
                        help='Override acceleration_weight (auto-detected from CSV/dir name if omitted)')
    parser.add_argument('--algorithm-name', type=str, default='I-MPC',
                        help='Algorithm name for LaTeX table (default: I-MPC)')
    parser.add_argument('--dynus-table-path', type=str,
                        default='/home/kkondo/paper_writing/DYNUS_v3/tables/dynamic_benchmark.tex',
                        help='Path to DYNUS LaTeX table')
    parser.add_argument('--skip-collision-postprocess', action='store_true',
                        help='Skip collision post-processing from rosbags')

    args = parser.parse_args()

    if args.config_dirs:
        analyze_multi_config(args.config_dirs, Path(args.dynus_table_path),
                             args.skip_collision_postprocess)
    elif args.multi_dir:
        analyze_multi(args.multi_dir, args.algorithm_name,
                      Path(args.dynus_table_path), args.skip_collision_postprocess)
    elif args.data_dir:
        analyze_single(Path(args.data_dir), args.algorithm_name,
                       Path(args.dynus_table_path), args.skip_collision_postprocess,
                       acceleration_weight=args.acceleration_weight)
    else:
        parser.error("Either --data-dir, --multi-dir, or --config-dirs is required")


if __name__ == '__main__':
    main()
