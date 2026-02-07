#!/usr/bin/env python3
"""
Post-process Intent-MPC benchmark data to check collisions from rosbags.

Similar to DYNUS collision checking, this script:
1. Reads rosbags recorded during trials
2. Extracts drone and obstacle trajectories
3. Checks for collisions using bounding box intersection
4. Updates the benchmark CSV with accurate collision data

Usage:
    python3 scripts/postprocess_collisions.py --data-dir data/benchmark_20260205_143512
"""

import argparse
import sys
from pathlib import Path
import pandas as pd
import numpy as np
import math

try:
    import rosbag
    HAS_ROSBAG = True
except ImportError:
    print("ERROR: rosbag not available. Install with: pip install --extra-index-url https://rospypi.github.io/simple/ rosbag")
    HAS_ROSBAG = False
    sys.exit(1)


class BoundingBox:
    """Axis-aligned bounding box for collision detection"""
    def __init__(self, min_x, max_x, min_y, max_y, min_z, max_z):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.min_z = min_z
        self.max_z = max_z

    @classmethod
    def from_center_and_half_extents(cls, cx, cy, cz, hx, hy, hz):
        return cls(cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz)

    def intersects(self, other):
        """Check if this AABB intersects with another AABB"""
        return not (self.max_x < other.min_x or self.min_x > other.max_x or
                    self.max_y < other.min_y or self.min_y > other.max_y or
                    self.max_z < other.min_z or self.min_z > other.max_z)

    def distance_to(self, other):
        """Calculate minimum distance between two AABBs"""
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
            x_str = name[10:13]
            y_str = name[14:17]
            z_str = name[18:21]
            x_size = float(x_str) / 100.0  # cm to m
            y_size = float(y_str) / 100.0
            z_size = float(z_str) / 100.0
            return (x_size / 2.0, y_size / 2.0, z_size / 2.0)  # Return half extents
        except:
            pass
    # Default size for dynamic obstacles (0.8m cube)
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

    # Linear interpolation
    idx = np.searchsorted(times, t_query)
    if idx >= len(times):
        return positions[-1]

    t0, t1 = times[idx-1], times[idx]
    p0, p1 = positions[idx-1], positions[idx]

    alpha = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
    return p0 + alpha * (p1 - p0)


def analyze_collision_from_bag(bag_path: Path, drone_bbox=(0.1, 0.1, 0.1)):
    """Analyze collisions from rosbag data

    Args:
        bag_path: Path to rosbag file
        drone_bbox: Drone half-extents (hx, hy, hz)

    Returns:
        Dictionary with collision statistics
    """
    if not bag_path.exists():
        print(f"  Warning: Bag not found at {bag_path}")
        return {'collision_count': 0, 'min_distance': float('inf'),
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0}

    print(f"  Analyzing: {bag_path.name}")

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as e:
        print(f"    Error opening bag: {e}")
        return {'collision_count': 0, 'min_distance': float('inf'),
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0}

    # Extract data
    drone_trajectory = []  # [(time, x, y, z), ...]
    obstacle_trajectories = {}  # {obs_name: [(time, x, y, z), ...]}
    obstacle_sizes = {}  # {obs_name: (hx, hy, hz)}

    try:
        # Read drone odometry
        for topic, msg, t in bag.read_messages(topics=['/CERLAB/quadcopter/odom']):
            timestamp = t.to_sec()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            drone_trajectory.append((timestamp, x, y, z))

        # Read obstacle states from Gazebo
        for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
            timestamp = t.to_sec()
            for i, name in enumerate(msg.name):
                if 'obstacle' in name:
                    pos = msg.pose[i].position
                    if name not in obstacle_trajectories:
                        obstacle_trajectories[name] = []
                        obstacle_sizes[name] = parse_obstacle_size(name)
                    obstacle_trajectories[name].append((timestamp, pos.x, pos.y, pos.z))

    finally:
        bag.close()

    if len(drone_trajectory) == 0:
        print(f"    Warning: No drone trajectory found in bag")
        return {'collision_count': 0, 'min_distance': float('inf'),
                'collision_free_ratio': 1.0, 'collision_penetration_max': 0.0}

    # Convert to numpy arrays
    drone_times = np.array([t for t, x, y, z in drone_trajectory])
    drone_positions = np.array([[x, y, z] for t, x, y, z in drone_trajectory])

    obstacle_data = {}
    for obs_name, traj in obstacle_trajectories.items():
        if len(traj) > 0:
            times = np.array([t for t, x, y, z in traj])
            positions = np.array([[x, y, z] for t, x, y, z in traj])
            obstacle_data[obs_name] = {
                'times': times,
                'positions': positions,
                'half_extents': obstacle_sizes[obs_name]
            }

    print(f"    Loaded {len(drone_positions)} drone positions, {len(obstacle_data)} obstacles")

    # Collision checking
    collisions = 0
    min_distance = float('inf')
    max_penetration = 0.0
    collision_free_segments = 0
    collided_obstacles = set()

    drone_hx, drone_hy, drone_hz = drone_bbox

    for i, (t, px, py, pz) in enumerate(zip(drone_times,
                                             drone_positions[:, 0],
                                             drone_positions[:, 1],
                                             drone_positions[:, 2])):
        drone_bbox_obj = BoundingBox.from_center_and_half_extents(px, py, pz, drone_hx, drone_hy, drone_hz)

        segment_collision_free = True

        for obs_name, obs_data in obstacle_data.items():
            # Interpolate obstacle position at this time
            obs_pos = interpolate_position(obs_data['times'], obs_data['positions'], t)
            obs_half_extents = obs_data['half_extents']

            obs_bbox = BoundingBox.from_center_and_half_extents(
                obs_pos[0], obs_pos[1], obs_pos[2],
                obs_half_extents[0], obs_half_extents[1], obs_half_extents[2]
            )

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

    collision_free_ratio = collision_free_segments / len(drone_positions) if len(drone_positions) > 0 else 1.0

    result = {
        'collision_count': collisions,
        'min_distance': min_distance if min_distance != float('inf') else 0.0,
        'collision_free_ratio': collision_free_ratio,
        'collision_penetration_max': max_penetration,
        'collision_unique_obstacles': len(collided_obstacles)
    }

    print(f"    Collisions: {collisions}, Min distance: {result['min_distance']:.3f}m, Unique obstacles hit: {len(collided_obstacles)}")
    return result


def postprocess_benchmark_data(data_dir: Path):
    """Post-process benchmark data to check collisions from rosbags"""

    print("="*80)
    print("INTENT-MPC COLLISION POST-PROCESSING")
    print("="*80)
    print()

    # Find benchmark CSV
    csv_files = list(data_dir.glob("benchmark_intent_mpc_*.csv"))
    csv_files = [f for f in csv_files if 'summary' not in f.name.lower()]

    if not csv_files:
        print(f"ERROR: No benchmark CSV found in {data_dir}")
        sys.exit(1)

    csv_file = sorted(csv_files, key=lambda f: f.stat().st_mtime)[-1]
    print(f"Loading benchmark data: {csv_file.name}")

    df = pd.read_csv(csv_file)
    print(f"  Trials: {len(df)}")
    print()

    # Find bags directory
    bags_dir = data_dir / "bags"
    if not bags_dir.exists():
        print(f"ERROR: No bags directory found at {bags_dir}")
        print(f"       Rosbags may not have been recorded")
        sys.exit(1)

    print(f"Processing rosbags from: {bags_dir}")
    print()

    # Process each trial
    for idx, row in df.iterrows():
        trial_id = row['trial_id']
        bag_path = bags_dir / f"trial_{trial_id}.bag"

        print(f"Trial {trial_id}:")

        if not bag_path.exists():
            print(f"  Warning: Bag file not found")
            continue

        # Analyze collisions
        collision_result = analyze_collision_from_bag(bag_path)

        # Update dataframe
        df.at[idx, 'collision_count'] = collision_result['collision_count']
        df.at[idx, 'min_distance_to_obstacles'] = collision_result['min_distance']
        df.at[idx, 'collision_free_ratio'] = collision_result['collision_free_ratio']
        df.at[idx, 'collision_penetration_max'] = collision_result['collision_penetration_max']
        df.at[idx, 'collision_unique_obstacles'] = collision_result['collision_unique_obstacles']
        df.at[idx, 'collision'] = collision_result['collision_count'] > 0

        print()

    # Save updated CSV
    output_csv = data_dir / f"benchmark_intent_mpc_postprocessed.csv"
    df.to_csv(output_csv, index=False)

    print("="*80)
    print("POST-PROCESSING COMPLETE")
    print("="*80)
    print(f"\nUpdated CSV saved to: {output_csv}")
    print()
    print("Summary:")
    print(f"  Trials processed: {len(df)}")
    print(f"  Trials with collisions: {(df['collision'] == True).sum()}")
    print(f"  Total collision events: {df['collision_count'].sum()}")
    print(f"  Avg min distance: {df['min_distance_to_obstacles'].mean():.3f}m")
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Post-process Intent-MPC benchmark collisions from rosbags',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument(
        '--data-dir',
        type=str,
        required=True,
        help='Path to benchmark data directory'
    )

    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    if not data_dir.exists():
        print(f"ERROR: Directory not found: {data_dir}")
        sys.exit(1)

    postprocess_benchmark_data(data_dir)


if __name__ == '__main__':
    main()
