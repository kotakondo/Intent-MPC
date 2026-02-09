#!/usr/bin/env python3
"""
Intent-MPC Benchmark Runner

Runs multiple trials of Intent-MPC with dynamic obstacles and collects performance data.
Adapted from DYNUS run_benchmark.py for ROS1 and Intent-MPC architecture.

Usage:
    # Inside Docker container
    python3 scripts/run_mpc_benchmark.py --num-trials 20 --num-obstacles 200 --dynamic-ratio 0.7

    # With custom output directory
    python3 scripts/run_mpc_benchmark.py --num-trials 20 --output-dir /root/ip-mpc_ws/src/Intent-MPC/data/experiment_1

    # Specific seed range
    python3 scripts/run_mpc_benchmark.py --num-trials 10 --seed-start 100
"""

import argparse
import csv
import json
import os
import subprocess
import signal
import sys
import time
from dataclasses import dataclass, asdict, field
from datetime import datetime
from pathlib import Path as FilePath  # Rename to avoid collision with nav_msgs.msg.Path
from typing import List, Tuple, Optional
import numpy as np
import math

# ROS1 imports
import rospy
import rosgraph
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

# Import tracking_controller Target message for commanded pos/vel/acc
try:
    from tracking_controller.msg import Target
    HAS_TARGET_MSG = True
except ImportError:
    HAS_TARGET_MSG = False
    rospy.logwarn("tracking_controller.msg.Target not available, will compute from odom")


@dataclass
class BenchmarkMetrics:
    """Comprehensive metrics for a single trial"""
    # Trial info
    trial_id: int = 0
    seed: int = 0
    num_obstacles: int = 0
    dynamic_ratio: float = 0.0
    timestamp: str = ""

    # Success metrics
    goal_reached: bool = False
    timeout_reached: bool = False
    collision: bool = False

    # Time metrics (seconds)
    flight_travel_time: float = 0.0
    total_replanning_time: float = 0.0

    # Replanning metrics
    num_replans: int = 0
    avg_replanning_time: float = 0.0  # ms
    max_replanning_time: float = 0.0  # ms

    # Path metrics
    path_length: float = 0.0  # meters
    straight_line_distance: float = 0.0  # meters
    path_efficiency: float = 0.0  # path_length / straight_line_distance

    # Smoothness metrics (lower is better)
    jerk_integral: float = 0.0  # integrated jerk over trajectory
    jerk_rms: float = 0.0  # RMS jerk
    avg_velocity: float = 0.0  # m/s
    max_velocity: float = 0.0  # m/s
    avg_acceleration: float = 0.0  # m/s²
    max_acceleration: float = 0.0  # m/s²

    # Constraint violations (DYNUS benchmark limits)
    vel_limit: float = 5.0  # m/s (DYNUS: v_max)
    acc_limit: float = 20.0  # m/s² (DYNUS: a_max)
    jerk_limit: float = 100.0  # m/s³ (DYNUS: j_max)

    vel_violation_count: int = 0
    acc_violation_count: int = 0
    jerk_violation_count: int = 0

    vel_violation_max: float = 0.0  # max velocity observed
    acc_violation_max: float = 0.0  # max acceleration observed
    jerk_violation_max: float = 0.0  # max jerk observed

    # MPC computation time metrics (seconds)
    mpc_compute_time_avg: float = 0.0  # Average computation time per solve
    mpc_compute_time_max: float = 0.0  # Maximum computation time
    mpc_compute_time_std: float = 0.0  # Standard deviation
    mpc_solve_count: int = 0  # Number of MPC solves

    # Collision metrics
    collision_count: int = 0
    collision_penetration_max: float = 0.0  # meters
    collision_unique_obstacles: int = 0
    min_distance_to_obstacles: float = float('inf')  # meters
    collision_free_ratio: float = 1.0  # fraction of trajectory without collisions

    # Goal configuration (DYNUS benchmark)
    start_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 2.0])
    goal_position: List[float] = field(default_factory=lambda: [105.0, 0.0, 2.0])

    # Rosbag path for post-processing
    bag_file: str = ""


class BenchmarkMonitor:
    """ROS1 node to monitor trial progress and collect metrics"""

    def __init__(self, trial_id: int, num_obstacles: int, dynamic_ratio: float, seed: int):
        self.metrics = BenchmarkMetrics(
            trial_id=trial_id,
            seed=seed,
            num_obstacles=num_obstacles,
            dynamic_ratio=dynamic_ratio,
            timestamp=datetime.now().strftime("%Y%m%d_%H%M%S")
        )

        # State tracking - ACTUAL positions from odometry (for path length)
        self.odom_data: List[Tuple[float, np.ndarray]] = []  # (time, position)
        self.start_time = None
        self.last_position = None
        self.last_time = None
        self.last_print_time = -10  # Track last print time to avoid spam

        # COMMANDED trajectory data from target_state (for vel/acc/jerk metrics)
        self.target_data: List[Tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []  # (time, pos, vel, acc)
        self.last_target_acc = None
        self.last_target_time = None

        # Goal tracking (DYNUS benchmark: (0,0,2) -> (105,0,2))
        self.goal_position = np.array([105.0, 0.0, 2.0])
        self.start_position = np.array([0.0, 0.0, 2.0])
        self.goal_threshold = 2.0  # meters
        self.start_threshold = 10.0  # meters - must be near start before monitoring begins
        self.waiting_for_start = True  # Don't start monitoring until drone is near start position
        self.monitor_created_time = time.time()  # Wall clock time for timeout
        self.odom_callback_count = 0  # Debug: count callbacks

        # Obstacle tracking for collision detection
        self.obstacle_positions = {}  # {name: position}
        self.obstacle_sizes = {}  # {name: (x_size, y_size, z_size)}

        # Drone bounding box (DYNUS benchmark: [0.2, 0.2, 0.2])
        self.drone_half_extents = (0.1, 0.1, 0.1)  # Half of [0.2, 0.2, 0.2]

        # ROS subscribers
        # Odom for actual position (path length calculation)
        self.odom_sub = rospy.Subscriber('/CERLAB/quadcopter/odom', Odometry, self.odom_callback)
        # Target state for commanded pos/vel/acc (like DYNUS /NX01/goal topic)
        if HAS_TARGET_MSG:
            self.target_sub = rospy.Subscriber('/autonomous_flight/target_state', Target, self.target_callback)
            rospy.loginfo("Subscribed to /autonomous_flight/target_state for commanded trajectory")
        # DYNUS obstacles publish to /dynus/model_states (NOT /gazebo/model_states)
        # Wait for topic to be available before subscribing
        dynus_topic = '/dynus/model_states'
        rospy.loginfo(f"Waiting for {dynus_topic} topic...")
        print(f"Waiting for {dynus_topic} topic...", flush=True)
        try:
            rospy.wait_for_message(dynus_topic, ModelStates, timeout=30.0)
            rospy.loginfo(f"Topic {dynus_topic} is available!")
            print(f"Topic {dynus_topic} is available!", flush=True)
        except rospy.ROSException as e:
            rospy.logwarn(f"Timeout waiting for {dynus_topic}: {e}")
            print(f"WARNING: Timeout waiting for {dynus_topic}", flush=True)

        self.model_states_sub = rospy.Subscriber(dynus_topic, ModelStates, self.model_states_callback)
        rospy.loginfo("Subscribed to /dynus/model_states for obstacle collision detection")

        # Status tracking
        self.is_complete = False
        self.completion_reason = None

        rospy.loginfo(f"Benchmark monitor initialized for trial {trial_id}")

    def odom_callback(self, msg: Odometry):
        """Collect ACTUAL position from odometry for path length calculation"""
        self.odom_callback_count += 1

        # Extract actual position from odometry
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        # Wait for drone to be near START position before beginning monitoring
        # This prevents false "goal reached" from stale data of previous trial
        if self.waiting_for_start:
            start_distance = np.linalg.norm(pos - self.start_position)
            elapsed_since_monitor_created = time.time() - self.monitor_created_time

            # Timeout: if we've been waiting more than 30 seconds, proceed anyway
            if elapsed_since_monitor_created > 30.0:
                rospy.logwarn(f"Trial {self.metrics.trial_id}: Timeout waiting for start position, proceeding anyway (pos: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}])")
                self.waiting_for_start = False
            elif start_distance > self.start_threshold:
                # Drone is not near start - likely stale data from previous trial
                rospy.loginfo_throttle(2.0, f"Trial {self.metrics.trial_id}: Waiting for drone near start (pos: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}], dist: {start_distance:.1f}m, callbacks: {self.odom_callback_count})")
                return
            else:
                # Drone is near start position - begin monitoring
                self.waiting_for_start = False
                rospy.loginfo(f"Trial {self.metrics.trial_id}: Drone near start position, beginning monitoring (callbacks so far: {self.odom_callback_count})")

        if self.start_time is None:
            self.start_time = rospy.Time.now().to_sec()
            rospy.loginfo(f"Trial {self.metrics.trial_id}: Started at {self.start_time}")

        current_time = rospy.Time.now().to_sec() - self.start_time

        # Store actual position for path length calculation
        self.odom_data.append((current_time, pos))

        self.last_position = pos
        self.last_time = current_time

        # Check if goal reached (with minimum flight time to avoid false positives)
        goal_distance = np.linalg.norm(pos - self.goal_position)
        min_flight_time = 10.0  # Goal can't be reached in less than 10 seconds (105m at 5m/s = 21s minimum)
        if goal_distance < self.goal_threshold and current_time > min_flight_time:
            if not self.is_complete:
                self.is_complete = True
                self.completion_reason = "goal_reached"
                self.metrics.goal_reached = True
                self.metrics.flight_travel_time = current_time
                rospy.loginfo(f"Trial {self.metrics.trial_id}: *** GOAL REACHED *** in {current_time:.2f}s (distance: {goal_distance:.2f}m)")

        # Debug output every 5 seconds (only once per interval)
        if current_time - self.last_print_time >= 5.0:
            self.last_print_time = current_time
            rospy.loginfo(f"Trial {self.metrics.trial_id}: t={current_time:.0f}s, pos=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}], goal_dist={goal_distance:.1f}m")

    def target_callback(self, msg):
        """Collect COMMANDED pos/vel/acc from target_state (like DYNUS /NX01/goal)"""
        if self.start_time is None:
            return  # Wait for odom to start timing

        current_time = rospy.Time.now().to_sec() - self.start_time

        # Extract commanded position, velocity, acceleration directly from message
        pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        acc = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z])

        # Store commanded trajectory data
        self.target_data.append((current_time, pos, vel, acc))

        self.last_target_acc = acc
        self.last_target_time = current_time

    def model_states_callback(self, msg: ModelStates):
        """Track obstacle positions for collision detection"""
        obstacles_found = 0
        for i, name in enumerate(msg.name):
            if 'obstacle' in name:
                obstacles_found += 1
                pos = msg.pose[i].position
                self.obstacle_positions[name] = np.array([pos.x, pos.y, pos.z])

                # Parse obstacle size from name (e.g., "obstacle_d080_080_080")
                if name not in self.obstacle_sizes:
                    size = self.parse_obstacle_size(name)
                    if size is not None:
                        self.obstacle_sizes[name] = size

        # Log obstacle count periodically (first time and every 500 callbacks)
        if not hasattr(self, '_model_states_callback_count'):
            self._model_states_callback_count = 0
        self._model_states_callback_count += 1
        if self._model_states_callback_count == 1:
            # First callback - print to both stdout and rospy
            print(f"*** DYNUS MODEL_STATES CALLBACK RECEIVED! Trial {self.metrics.trial_id}: tracking {len(self.obstacle_positions)} obstacles ***", flush=True)
            rospy.loginfo(f"Trial {self.metrics.trial_id}: model_states callback #{self._model_states_callback_count}, tracking {len(self.obstacle_positions)} obstacles")
        elif self._model_states_callback_count % 500 == 0:
            rospy.loginfo(f"Trial {self.metrics.trial_id}: model_states callback #{self._model_states_callback_count}, tracking {len(self.obstacle_positions)} obstacles")

        # Check for collisions if we have odometry data
        if self.last_position is not None:
            self.check_collisions()

    def parse_obstacle_size(self, name: str) -> Optional[Tuple[float, float, float]]:
        """Parse obstacle size from name (format: obstacle_dXXX_YYY_ZZZ)"""
        if len(name) == 21 and name.startswith("obstacle_d"):
            try:
                x_str = name[10:13]
                y_str = name[14:17]
                z_str = name[18:21]
                x_size = float(x_str) / 100.0  # cm to m
                y_size = float(y_str) / 100.0
                z_size = float(z_str) / 100.0
                return (x_size, y_size, z_size)
            except:
                pass
        # Default size for dynamic obstacles
        return (0.8, 0.8, 0.8)

    def check_collisions(self):
        """Check for collisions with all obstacles"""
        if self.last_position is None:
            return

        drone_bbox = self.create_bbox(self.last_position, self.drone_half_extents)

        for obs_name, obs_pos in self.obstacle_positions.items():
            obs_size = self.obstacle_sizes.get(obs_name, (0.8, 0.8, 0.8))
            obs_half_extents = tuple(s / 2.0 for s in obs_size)
            obs_bbox = self.create_bbox(obs_pos, obs_half_extents)

            # Check intersection
            distance = self.bbox_distance(drone_bbox, obs_bbox)
            self.metrics.min_distance_to_obstacles = min(self.metrics.min_distance_to_obstacles, distance)

            if self.bbox_intersects(drone_bbox, obs_bbox):
                self.metrics.collision_count += 1
                penetration = -distance  # Negative distance = penetration
                self.metrics.collision_penetration_max = max(self.metrics.collision_penetration_max, penetration)

                if not self.metrics.collision:
                    self.metrics.collision = True
                    rospy.logwarn(f"Trial {self.metrics.trial_id}: Collision detected with {obs_name}")

    @staticmethod
    def create_bbox(center: np.ndarray, half_extents: Tuple[float, float, float]) -> dict:
        """Create axis-aligned bounding box"""
        return {
            'min_x': center[0] - half_extents[0],
            'max_x': center[0] + half_extents[0],
            'min_y': center[1] - half_extents[1],
            'max_y': center[1] + half_extents[1],
            'min_z': center[2] - half_extents[2],
            'max_z': center[2] + half_extents[2],
        }

    @staticmethod
    def bbox_intersects(bbox1: dict, bbox2: dict) -> bool:
        """Check if two bounding boxes intersect"""
        return not (bbox1['max_x'] < bbox2['min_x'] or bbox1['min_x'] > bbox2['max_x'] or
                    bbox1['max_y'] < bbox2['min_y'] or bbox1['min_y'] > bbox2['max_y'] or
                    bbox1['max_z'] < bbox2['min_z'] or bbox1['min_z'] > bbox2['max_z'])

    @staticmethod
    def bbox_distance(bbox1: dict, bbox2: dict) -> float:
        """Calculate minimum distance between two bounding boxes"""
        if BenchmarkMonitor.bbox_intersects(bbox1, bbox2):
            return 0.0
        dx = max(0.0, max(bbox1['min_x'], bbox2['min_x']) - min(bbox1['max_x'], bbox2['max_x']))
        dy = max(0.0, max(bbox1['min_y'], bbox2['min_y']) - min(bbox1['max_y'], bbox2['max_y']))
        dz = max(0.0, max(bbox1['min_z'], bbox2['min_z']) - min(bbox1['max_z'], bbox2['max_z']))
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def compute_final_metrics(self):
        """Compute all metrics after trial completion

        Uses:
        - odom_data: ACTUAL positions for path length (like DYNUS)
        - target_data: COMMANDED pos/vel/acc for velocity/acceleration/jerk metrics (like DYNUS /NX01/goal)
        """
        if len(self.odom_data) == 0:
            rospy.logwarn(f"Trial {self.metrics.trial_id}: No odometry data collected")
            return

        rospy.loginfo(f"Trial {self.metrics.trial_id}: Computing metrics from {len(self.odom_data)} odom points, {len(self.target_data)} target points")

        # ========================================
        # PATH LENGTH from ACTUAL positions (odom)
        # ========================================
        odom_times = np.array([t for t, _ in self.odom_data])
        positions = np.array([p for _, p in self.odom_data])

        # Path length (sum of distances between consecutive actual positions)
        if len(positions) > 1:
            diffs = np.diff(positions, axis=0)
            segment_lengths = np.linalg.norm(diffs, axis=1)
            self.metrics.path_length = float(np.sum(segment_lengths))

        # Straight line distance (from actual start to actual end position)
        actual_start = positions[0]
        actual_end = positions[-1]
        self.metrics.straight_line_distance = float(np.linalg.norm(actual_end - actual_start))

        # Debug: log actual vs expected positions
        rospy.loginfo(f"Trial {self.metrics.trial_id}: Actual start: [{actual_start[0]:.2f}, {actual_start[1]:.2f}, {actual_start[2]:.2f}]")
        rospy.loginfo(f"Trial {self.metrics.trial_id}: Actual end: [{actual_end[0]:.2f}, {actual_end[1]:.2f}, {actual_end[2]:.2f}]")
        rospy.loginfo(f"Trial {self.metrics.trial_id}: Expected start: {self.start_position}, Expected goal: {self.goal_position}")
        rospy.loginfo(f"Trial {self.metrics.trial_id}: Path length: {self.metrics.path_length:.2f}m, Straight line: {self.metrics.straight_line_distance:.2f}m")

        # Path efficiency
        if self.metrics.straight_line_distance > 0:
            self.metrics.path_efficiency = self.metrics.path_length / self.metrics.straight_line_distance

        # ========================================
        # VELOCITY/ACCELERATION/JERK from COMMANDED trajectory (target_state)
        # This is like DYNUS using /NX01/goal topic
        # ========================================
        if len(self.target_data) > 0:
            target_times = np.array([t for t, _, _, _ in self.target_data])
            target_positions = np.array([p for _, p, _, _ in self.target_data])
            velocities = np.array([v for _, _, v, _ in self.target_data])
            accelerations = np.array([a for _, _, _, a in self.target_data])

            # Velocity metrics from commanded trajectory
            vel_magnitudes = np.linalg.norm(velocities, axis=1)
            valid_vel_mask = vel_magnitudes > 0.01  # Filter out near-zero velocities
            if np.any(valid_vel_mask):
                valid_vel = vel_magnitudes[valid_vel_mask]
                self.metrics.avg_velocity = float(np.mean(valid_vel))
                self.metrics.max_velocity = float(np.max(valid_vel))
            else:
                self.metrics.avg_velocity = 0.0
                self.metrics.max_velocity = 0.0

            rospy.loginfo(f"Trial {self.metrics.trial_id}: Commanded Velocity - avg: {self.metrics.avg_velocity:.2f} m/s, max: {self.metrics.max_velocity:.2f} m/s (limit: {self.metrics.vel_limit} m/s)")

            # Velocity violations (per-component, like DYNUS)
            for vel in velocities:
                for component in vel:
                    if abs(component) > self.metrics.vel_limit + 1e-3:  # Small tolerance
                        self.metrics.vel_violation_count += 1
                        excess = abs(component) - self.metrics.vel_limit
                        self.metrics.vel_violation_max = max(self.metrics.vel_violation_max, abs(component))

            if self.metrics.vel_violation_count > 0:
                rospy.logwarn(f"Trial {self.metrics.trial_id}: VELOCITY VIOLATIONS: {self.metrics.vel_violation_count} component violations (max: {self.metrics.vel_violation_max:.2f} m/s)")

            # Acceleration metrics from commanded trajectory
            acc_magnitudes = np.linalg.norm(accelerations, axis=1)
            self.metrics.avg_acceleration = float(np.mean(acc_magnitudes))
            self.metrics.max_acceleration = float(np.max(acc_magnitudes))

            rospy.loginfo(f"Trial {self.metrics.trial_id}: Commanded Acceleration - avg: {self.metrics.avg_acceleration:.2f} m/s², max: {self.metrics.max_acceleration:.2f} m/s² (limit: {self.metrics.acc_limit} m/s²)")

            # Acceleration violations (per-component, like DYNUS)
            for acc in accelerations:
                for component in acc:
                    if abs(component) > self.metrics.acc_limit + 1e-3:
                        self.metrics.acc_violation_count += 1
                        self.metrics.acc_violation_max = max(self.metrics.acc_violation_max, abs(component))

            # Jerk computation (derivative of acceleration from commanded trajectory)
            # Use min length to avoid index out of bounds
            min_len = min(len(accelerations), len(target_times))
            if min_len > 1:
                jerks = []
                jerk_vectors = []
                for i in range(1, min_len):
                    dt = target_times[i] - target_times[i-1]
                    if dt > 0.001:  # Avoid division by very small dt
                        jerk = (accelerations[i] - accelerations[i-1]) / dt
                        jerk_vectors.append(jerk)
                        jerk_mag = np.linalg.norm(jerk)
                        jerks.append(jerk_mag)

                if len(jerks) > 0:
                    jerks = np.array(jerks)
                    self.metrics.jerk_rms = float(np.sqrt(np.mean(jerks**2)))
                    # Jerk integral (like DYNUS: sum of magnitudes * avg dt)
                    avg_dt = np.mean(np.diff(target_times))
                    self.metrics.jerk_integral = float(np.sum(jerks) * avg_dt)

                    rospy.loginfo(f"Trial {self.metrics.trial_id}: Jerk - RMS: {self.metrics.jerk_rms:.2f} m/s³, Integral: {self.metrics.jerk_integral:.2f} (limit: {self.metrics.jerk_limit} m/s³)")

                    # Jerk violations (per-component, like DYNUS)
                    for jerk in jerk_vectors:
                        for component in jerk:
                            if abs(component) > self.metrics.jerk_limit + 1e-3:
                                self.metrics.jerk_violation_count += 1
                                self.metrics.jerk_violation_max = max(self.metrics.jerk_violation_max, abs(component))

        else:
            rospy.logwarn(f"Trial {self.metrics.trial_id}: No target_state data - velocity/acceleration metrics unavailable")
            rospy.logwarn(f"  Make sure /autonomous_flight/target_state is being published")

        # Collision-free ratio
        if self.metrics.collision_count > 0:
            total_points = len(self.odom_data)
            collision_points = min(self.metrics.collision_count, total_points)
            self.metrics.collision_free_ratio = 1.0 - (collision_points / total_points)

        # Count unique obstacles hit
        if self.metrics.collision_count > 0:
            self.metrics.collision_unique_obstacles = max(1, int(self.metrics.collision_count / 10))

        rospy.loginfo(f"Trial {self.metrics.trial_id}: Metrics computed successfully")
        rospy.loginfo(f"  Path length: {self.metrics.path_length:.2f}m")
        rospy.loginfo(f"  Flight time: {self.metrics.flight_travel_time:.2f}s")
        rospy.loginfo(f"  Collisions: {self.metrics.collision_count}")
        rospy.loginfo(f"  Obstacles tracked: {len(self.obstacle_positions)}")
        rospy.loginfo(f"  Model states callbacks received: {getattr(self, '_model_states_callback_count', 0)}")
        min_dist = self.metrics.min_distance_to_obstacles
        if min_dist == float('inf'):
            rospy.logwarn(f"  Min distance to obstacles: NO OBSTACLES DETECTED (inf) - check /dynus/model_states topic!")
            # Set to -1 to indicate no data (instead of inf which causes CSV issues)
            self.metrics.min_distance_to_obstacles = -1.0
        else:
            rospy.loginfo(f"  Min distance to obstacles: {min_dist:.3f}m")


def wait_for_ros_master(timeout=30):
    """Wait for ROS master to be available"""
    print("Waiting for ROS master to start...", flush=True)
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            # Try to get master URI
            import rosgraph
            master = rosgraph.Master('/rostopic')
            master.getPid()
            print("✓ ROS master is ready", flush=True)
            return True
        except:
            time.sleep(0.5)

    print("✗ ROS master did not start within timeout", flush=True)
    return False


def start_rosbag_recording(bag_path: FilePath, topics: List[str]):
    """Start recording rosbag for post-processing"""
    bag_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = ['rosbag', 'record', '-O', str(bag_path)] + topics

    print(f"Starting rosbag recording: {bag_path.name}", flush=True)
    print(f"  Topics: {', '.join(topics)}", flush=True)

    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    time.sleep(2)  # Give rosbag time to start

    return proc


def kill_all_ros_processes(max_retries=3, keep_roscore=False):
    """Kill ROS and Gazebo processes between trials

    Args:
        max_retries: Number of cleanup attempts
        keep_roscore: If True, keeps rosmaster/roscore alive (needed for multi-trial runs
                      because rospy can only init_node once per Python process)
    """
    print(f"Killing ROS/Gazebo processes (keep_roscore={keep_roscore})...", flush=True)

    # Phase 1: Kill Intent-MPC specific nodes first (these can have cached state)
    intent_mpc_nodes = [
        'tracking_controller_node',
        'mpc_navigation_node',
        'fake_detector_node',
        'dynamic_map_node',
        'predictor_node',
        'spawn_model',
        'spawn_gazebo_model',
    ]

    print("  Phase 1: Killing Intent-MPC nodes...", flush=True)
    for node_name in intent_mpc_nodes:
        try:
            subprocess.run(['killall', '-9', node_name],
                          stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        except:
            pass

    # Also use pkill to catch any python nodes with these patterns
    python_patterns = ['mpc_navigation', 'tracking_controller', 'fake_detector', 'dynus']
    for pattern in python_patterns:
        try:
            subprocess.run(['pkill', '-9', '-f', pattern],
                          stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        except:
            pass

    time.sleep(1)

    # Phase 2: Kill Gazebo and general ROS processes
    processes_to_kill = [
        'gzserver',
        'gzclient',
        'roslaunch',
        'rosout',
        'rviz',
        'rosbag',
    ]

    # Only kill rosmaster/roscore if explicitly requested
    if not keep_roscore:
        processes_to_kill.extend(['rosmaster', 'roscore'])

    print("  Phase 2: Killing Gazebo/ROS processes...", flush=True)
    for attempt in range(max_retries):
        # Kill using killall (matches exact process name only)
        for proc_name in processes_to_kill:
            try:
                subprocess.run(['killall', '-9', proc_name],
                              stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
            except:
                pass

        # Wait for processes to die
        time.sleep(2)

        # Check if main processes are dead
        gz_alive = subprocess.run(['pgrep', 'gzserver'],
                                  capture_output=True).returncode == 0
        roslaunch_alive = subprocess.run(['pgrep', '-x', 'roslaunch'],
                                         capture_output=True).returncode == 0

        if not gz_alive and not roslaunch_alive:
            print(f"  Cleanup successful", flush=True)
            break
        elif attempt < max_retries - 1:
            print(f"  Retrying cleanup (gz={gz_alive}, roslaunch={roslaunch_alive})...", flush=True)

    # Phase 3: Clean up dead nodes from rosmaster (if keeping roscore)
    if keep_roscore:
        print("  Phase 3: Cleaning up rosmaster...", flush=True)
        try:
            # rosnode cleanup removes dead nodes from the master
            cleanup_proc = subprocess.Popen(
                ['rosnode', 'cleanup'],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            # Send 'y' to confirm cleanup
            cleanup_proc.communicate(input=b'y\n', timeout=5)
        except Exception as e:
            pass  # Ignore errors, cleanup is best-effort

    # Final wait to ensure everything is fully cleaned up
    time.sleep(3)
    print("Cleanup complete", flush=True)


def launch_simulation_and_navigation(num_obstacles: int, dynamic_ratio: float, seed: int, visualize: bool = False):
    """Launch Intent-MPC with DYNUS obstacles using split launch (like working run-dynamic-gazebo)

    This mimics the exact tmuxp approach from dynus_sim.yml:
    1. Start roscore first (separate process)
    2. Wait 3 seconds
    3. Launch Gazebo + obstacles (start_dynus.launch) with gui:=false
    4. Wait 5 more seconds (8 total from start)
    5. Send takeoff command
    6. Launch navigation (intent_mpc_dynus_nav.launch)
    """
    print(f"Launching Intent-MPC with DYNUS obstacles (split launch):", flush=True)
    print(f"  Obstacles: {num_obstacles}", flush=True)
    print(f"  Dynamic ratio: {dynamic_ratio}", flush=True)
    print(f"  Seed: {seed}", flush=True)
    print(f"  Visualization: {'RViz' if visualize else 'Headless'}", flush=True)

    # STEP 1: Start roscore first (if not already running)
    roscore_proc = None
    roscore_already_running = subprocess.run(['pgrep', 'rosmaster'], capture_output=True).returncode == 0

    if roscore_already_running:
        print(f"Step 1: roscore already running, reusing it", flush=True)
    else:
        print(f"Step 1: Starting roscore...", flush=True)
        roscore_proc = subprocess.Popen(['roscore'], preexec_fn=os.setsid,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        # Wait for ROS master (like tmuxp sleep 3)
        print("Step 2: Waiting for roscore (3s)...", flush=True)
        time.sleep(3)

    if not wait_for_ros_master(timeout=10):
        print("ERROR: ROS master failed to start", flush=True)
        if roscore_proc:
            roscore_proc.kill()
        return None

    # STEP 3: Launch Gazebo + DYNUS obstacles (like tmuxp pane 2)
    # IMPORTANT: gui:=false to disable Gazebo GUI
    gazebo_cmd = [
        'roslaunch',
        'uav_simulator',
        'start_dynus.launch',
        f'num_obstacles:={num_obstacles}',
        f'dynamic_ratio:={dynamic_ratio}',
        f'seed:={seed}',
        'gui:=false'  # Always headless for Gazebo (RViz is separate)
    ]

    print(f"Step 3: Launching Gazebo + obstacles (headless)...", flush=True)
    print(f"  Command: {' '.join(gazebo_cmd)}", flush=True)

    gazebo_proc = subprocess.Popen(gazebo_cmd, preexec_fn=os.setsid,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # Store roscore proc so we can kill it later
    gazebo_proc.roscore_proc = roscore_proc
    gazebo_proc.nav_proc = None

    # Wait for Gazebo to fully initialize
    print("Step 4: Waiting for Gazebo to initialize (8s)...", flush=True)
    time.sleep(8)

    # STEP 4b: Verify drone is at start position (0, 0, ~0.1 before takeoff)
    print("Step 4b: Verifying drone spawn position...", flush=True)
    try:
        result = subprocess.run(
            ['rostopic', 'echo', '-n', '1', '/CERLAB/quadcopter/odom'],
            timeout=10, capture_output=True, text=True
        )
        if 'position' in result.stdout:
            # Parse position from output
            lines = result.stdout.split('\n')
            for i, line in enumerate(lines):
                if 'x:' in line and i < 10:  # Look in first few lines (position section)
                    try:
                        x_val = float(line.split(':')[1].strip())
                        if abs(x_val) < 5.0:  # Should be near origin
                            print(f"  Drone position OK (x={x_val:.1f})", flush=True)
                        else:
                            print(f"  WARNING: Drone not at start! x={x_val:.1f}", flush=True)
                        break
                    except:
                        pass
    except Exception as e:
        print(f"  Warning: Could not verify position: {e}", flush=True)

    # STEP 5: Send takeoff command (like tmuxp pane 3 first command)
    print("Step 5: Sending takeoff command...", flush=True)
    takeoff_cmd = ['rostopic', 'pub', '-1', '/CERLAB/quadcopter/takeoff', 'std_msgs/Empty']
    try:
        result = subprocess.run(takeoff_cmd, timeout=10, capture_output=True, text=True)
        if result.returncode == 0:
            print("  Takeoff command sent successfully", flush=True)
        else:
            print(f"  Takeoff command returned: {result.returncode}", flush=True)
    except subprocess.TimeoutExpired:
        print(f"  Warning: Takeoff command timed out (this may be OK)", flush=True)
    except Exception as e:
        print(f"  Warning: Takeoff command error: {e}", flush=True)

    # Wait for takeoff to complete (drone needs time to reach 2m height)
    print("Step 5b: Waiting for takeoff to complete (5s)...", flush=True)
    time.sleep(5)

    # STEP 6: Launch navigation nodes (like tmuxp pane 3 second command)
    nav_cmd = [
        'roslaunch',
        'autonomous_flight',
        'intent_mpc_dynus_nav.launch'
    ]

    print(f"Step 6: Launching navigation...", flush=True)
    print(f"  Command: {' '.join(nav_cmd)}", flush=True)

    nav_proc = subprocess.Popen(nav_cmd, preexec_fn=os.setsid,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # Store nav_proc so we can kill it later
    gazebo_proc.nav_proc = nav_proc

    # Wait for navigation to initialize
    print("Step 7: Waiting for navigation to initialize (5s)...", flush=True)
    time.sleep(5)

    print("All systems launched successfully", flush=True)
    return gazebo_proc


def run_trial(trial_id: int, num_obstacles: int, dynamic_ratio: float, seed: int,
              timeout: float = 120.0, visualize: bool = False, output_dir: FilePath = None) -> BenchmarkMetrics:
    """Run a single benchmark trial"""

    print("="*80, flush=True)
    print(f"TRIAL {trial_id}: seed={seed}, obstacles={num_obstacles}, ratio={dynamic_ratio}", flush=True)
    print("="*80, flush=True)

    # PRE-TRIAL CLEANUP: Kill any lingering processes from previous trials
    # Keep roscore alive - rospy can only init_node once per Python process
    if trial_id > 0:
        print(f"Trial {trial_id}: Pre-trial cleanup (keeping roscore)...", flush=True)
        kill_all_ros_processes(keep_roscore=True)
    else:
        print(f"Trial {trial_id}: First trial, skipping pre-cleanup", flush=True)

    # FIRST: Launch simulation and navigation (this starts roscore)
    sim_nav_proc = launch_simulation_and_navigation(num_obstacles, dynamic_ratio, seed, visualize)

    if sim_nav_proc is None:
        print(f"ERROR: Failed to launch simulation for trial {trial_id}", flush=True)
        return BenchmarkMetrics(trial_id=trial_id, seed=seed, num_obstacles=num_obstacles, dynamic_ratio=dynamic_ratio)

    # SECOND: Initialize ROS node (now that roscore is running)
    try:
        rospy.init_node(f'benchmark_trial_{trial_id}', anonymous=True, disable_signals=True)
        print("✓ ROS node initialized", flush=True)
    except rospy.exceptions.ROSException as e:
        # Node already initialized from previous trial, this is OK
        print(f"ROS node already initialized, continuing with trial {trial_id}", flush=True)

    # THIRD: Create monitor (which subscribes to topics)
    monitor = BenchmarkMonitor(trial_id, num_obstacles, dynamic_ratio, seed)

    # FOURTH: Start rosbag recording for post-processing
    if output_dir is None:
        output_dir = FilePath('/root/ip-mpc_ws/src/Intent-MPC/data')

    bag_dir = output_dir / "bags"
    bag_path = bag_dir / f"trial_{trial_id}.bag"

    # Topics to record for collision checking
    topics_to_record = [
        '/CERLAB/quadcopter/odom',           # Drone odometry (actual position for path length)
        '/autonomous_flight/target_state',   # Commanded pos/vel/acc (like DYNUS /NX01/goal)
        '/dynus/model_states',               # DYNUS obstacle positions (NOT /gazebo/model_states!)
        '/mpcNavigation/mpc_trajectory',     # MPC trajectory
        '/mpcNavigation/mpc_compute_time',   # MPC computation time per solve
    ]

    rosbag_proc = start_rosbag_recording(bag_path, topics_to_record)

    # Monitor trial
    start_time = time.time()
    rate = rospy.Rate(5)  # 5 Hz monitoring (faster check)

    rospy.loginfo(f"Trial {trial_id}: Monitoring started, goal position: {monitor.goal_position}")

    try:
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time

            # Check timeout
            if elapsed > timeout:
                rospy.logwarn(f"Trial {trial_id}: TIMEOUT after {elapsed:.1f}s")
                monitor.metrics.timeout_reached = True
                monitor.metrics.flight_travel_time = elapsed
                break

            # Check completion
            if monitor.is_complete:
                rospy.loginfo(f"Trial {trial_id}: *** COMPLETED *** ({monitor.completion_reason})")
                # Add grace period to collect final data
                rospy.loginfo(f"Trial {trial_id}: Waiting 2s grace period for data collection...")
                time.sleep(2)
                break

            # Status update every 5 seconds
            if int(elapsed) % 5 == 0 and elapsed > 0 and abs(elapsed - int(elapsed)) < 0.2:
                if monitor.last_position is not None:
                    goal_dist = np.linalg.norm(monitor.last_position - monitor.goal_position)
                    status_msg = f"Trial {trial_id}: t={elapsed:.0f}s, pos=[{monitor.last_position[0]:.1f}, {monitor.last_position[1]:.1f}, {monitor.last_position[2]:.1f}], goal_dist={goal_dist:.2f}m, points={len(monitor.odom_data)}, callbacks={monitor.odom_callback_count}"
                    print(status_msg, flush=True)
                    rospy.loginfo(status_msg)
                else:
                    # No position yet - callbacks might not be firing
                    status_msg = f"Trial {trial_id}: t={elapsed:.0f}s, NO POSITION YET, callbacks={monitor.odom_callback_count}, waiting_for_start={monitor.waiting_for_start}"
                    print(status_msg, flush=True)
                    rospy.logwarn(status_msg)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.logwarn(f"Trial {trial_id}: Interrupted by user")

    finally:
        # Stop rosbag recording first
        print(f"Trial {trial_id}: Stopping rosbag recording...")
        try:
            rosbag_proc.send_signal(signal.SIGINT)  # Graceful stop
            rosbag_proc.wait(timeout=5)
            print(f"Trial {trial_id}: Rosbag saved to {bag_path}")
        except:
            rosbag_proc.kill()

        # Compute final metrics
        print(f"Trial {trial_id}: Computing metrics...")
        monitor.compute_final_metrics()

        # Store bag path in metrics for post-processing
        monitor.metrics.bag_file = str(bag_path)

        # Unregister subscribers before cleanup (don't call signal_shutdown - breaks reinitialization)
        print(f"Trial {trial_id}: Unregistering ROS subscribers...")
        try:
            monitor.odom_sub.unregister()
            if hasattr(monitor, 'target_sub'):
                monitor.target_sub.unregister()
            monitor.model_states_sub.unregister()
        except Exception as e:
            print(f"  Warning: Failed to unregister subscribers: {e}")

        # POST-TRIAL CLEANUP: Kill all ROS/Gazebo processes comprehensively
        print(f"Trial {trial_id}: Post-trial cleanup...")

        # Phase 1: Graceful shutdown (SIGTERM)
        print(f"  Phase 1: Graceful shutdown...", flush=True)

        # Kill navigation first (has cached trajectories/states)
        if hasattr(sim_nav_proc, 'nav_proc') and sim_nav_proc.nav_proc is not None:
            try:
                os.killpg(os.getpgid(sim_nav_proc.nav_proc.pid), signal.SIGTERM)
            except:
                pass

        # Kill gazebo
        try:
            os.killpg(os.getpgid(sim_nav_proc.pid), signal.SIGTERM)
        except:
            pass

        time.sleep(3)

        # Phase 2: Force kill if still alive (SIGKILL)
        print(f"  Phase 2: Force kill remaining processes...", flush=True)
        if hasattr(sim_nav_proc, 'nav_proc') and sim_nav_proc.nav_proc is not None:
            try:
                os.killpg(os.getpgid(sim_nav_proc.nav_proc.pid), signal.SIGKILL)
            except:
                pass

        try:
            os.killpg(os.getpgid(sim_nav_proc.pid), signal.SIGKILL)
        except:
            pass

        time.sleep(2)

        # Phase 3: Comprehensive cleanup (keeps roscore for next trial)
        print(f"  Phase 3: Comprehensive cleanup...", flush=True)
        kill_all_ros_processes(keep_roscore=True)

        # Phase 4: Verify cleanup and wait
        print(f"  Phase 4: Verifying cleanup...", flush=True)
        for _ in range(3):
            gz_alive = subprocess.run(['pgrep', 'gzserver'], capture_output=True).returncode == 0
            nav_alive = subprocess.run(['pgrep', '-f', 'mpc_navigation'], capture_output=True).returncode == 0
            ctrl_alive = subprocess.run(['pgrep', '-f', 'tracking_controller'], capture_output=True).returncode == 0

            if not gz_alive and not nav_alive and not ctrl_alive:
                print(f"  Verification passed: all processes dead", flush=True)
                break
            else:
                print(f"  Still alive: gz={gz_alive}, nav={nav_alive}, ctrl={ctrl_alive}", flush=True)
                kill_all_ros_processes(keep_roscore=True)
                time.sleep(2)

        # Final wait to ensure everything is fully cleaned up
        time.sleep(5)
        print(f"Trial {trial_id}: Cleanup complete")

    return monitor.metrics


def save_metrics_csv(metrics_list: List[BenchmarkMetrics], output_path: FilePath):
    """Save metrics to CSV file"""
    if not metrics_list:
        print("WARNING: No metrics to save")
        return

    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Convert dataclasses to dicts, filtering out nested lists
        rows = []
        for m in metrics_list:
            row = asdict(m)
            # Convert list fields to strings for CSV
            row['start_position'] = str(row['start_position'])
            row['goal_position'] = str(row['goal_position'])
            rows.append(row)

        # Write CSV
        with open(output_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

        print(f"\n✓ Metrics saved to: {output_path}")
    except Exception as e:
        print(f"ERROR saving CSV: {e}")
        import traceback
        traceback.print_exc()


def save_metrics_json(metrics_list: List[BenchmarkMetrics], output_path: FilePath):
    """Save metrics to JSON file"""
    if not metrics_list:
        return

    output_path.parent.mkdir(parents=True, exist_ok=True)

    data = [asdict(m) for m in metrics_list]

    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"✓ Metrics saved to: {output_path}")


def run_debug_trial(num_obstacles: int, dynamic_ratio: float, seed: int):
    """Run a single trial in debug mode with tmux panes for monitoring.

    Creates a tmux session with:
    - Pane 0: Gazebo + obstacles launch
    - Pane 1: Navigation launch
    - Pane 2: Odom monitoring (rostopic echo)
    - Pane 3: Odom frequency (rostopic hz)
    - Pane 4: Topic info (rostopic info)
    """
    print("="*80)
    print("DEBUG MODE - Single trial with tmux monitoring")
    print("="*80)
    print(f"  Obstacles: {num_obstacles}")
    print(f"  Dynamic ratio: {dynamic_ratio}")
    print(f"  Seed: {seed}")
    print()

    session_name = "intent_mpc_debug"

    # Kill any existing debug session
    subprocess.run(['tmux', 'kill-session', '-t', session_name],
                   stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

    # Kill any existing ROS/Gazebo processes
    kill_all_ros_processes(keep_roscore=False)
    time.sleep(2)

    # Create new tmux session
    subprocess.run(['tmux', 'new-session', '-d', '-s', session_name, '-n', 'main'])

    # Source ROS in all panes
    ros_source = "source /opt/ros/noetic/setup.bash && source /root/ip-mpc_ws/devel/setup.bash && source /root/ip-mpc_ws/src/Intent-MPC/uav_simulator/gazeboSetup.bash"

    # Pane 0 (main): Start roscore, then Gazebo
    gazebo_cmd = f"{ros_source} && roscore & sleep 3 && roslaunch uav_simulator start_dynus.launch num_obstacles:={num_obstacles} dynamic_ratio:={dynamic_ratio} seed:={seed} gui:=false"
    subprocess.run(['tmux', 'send-keys', '-t', f'{session_name}:0.0', gazebo_cmd, 'Enter'])

    # Wait for Gazebo to start
    print("Waiting 10s for Gazebo to initialize...")
    time.sleep(10)

    # Split horizontally for pane 1 (navigation)
    subprocess.run(['tmux', 'split-window', '-h', '-t', f'{session_name}:0'])
    nav_cmd = f"{ros_source} && sleep 3 && rostopic pub -1 /CERLAB/quadcopter/takeoff std_msgs/Empty && sleep 3 && roslaunch autonomous_flight intent_mpc_dynus_nav.launch"
    subprocess.run(['tmux', 'send-keys', '-t', f'{session_name}:0.1', nav_cmd, 'Enter'])

    # Split pane 0 vertically for odom echo
    subprocess.run(['tmux', 'split-window', '-v', '-t', f'{session_name}:0.0'])
    odom_echo_cmd = f"{ros_source} && echo 'Waiting for odom...' && sleep 5 && rostopic echo /CERLAB/quadcopter/odom/pose/pose/position"
    subprocess.run(['tmux', 'send-keys', '-t', f'{session_name}:0.2', odom_echo_cmd, 'Enter'])

    # Split pane 1 vertically for odom hz
    subprocess.run(['tmux', 'split-window', '-v', '-t', f'{session_name}:0.1'])
    odom_hz_cmd = f"{ros_source} && echo 'Waiting for odom_raw...' && sleep 5 && rostopic hz /CERLAB/quadcopter/odom_raw"
    subprocess.run(['tmux', 'send-keys', '-t', f'{session_name}:0.3', odom_hz_cmd, 'Enter'])

    # Add a 5th pane for topic info
    subprocess.run(['tmux', 'split-window', '-v', '-t', f'{session_name}:0.2'])
    info_cmd = f"{ros_source} && sleep 8 && echo '=== ODOM PUBLISHERS ===' && rostopic info /CERLAB/quadcopter/odom && echo '' && echo '=== ODOM_RAW PUBLISHERS ===' && rostopic info /CERLAB/quadcopter/odom_raw && echo '' && echo 'Press Enter to refresh...' && while true; do read; rostopic info /CERLAB/quadcopter/odom; done"
    subprocess.run(['tmux', 'send-keys', '-t', f'{session_name}:0.4', info_cmd, 'Enter'])

    print()
    print("="*80)
    print("DEBUG SESSION STARTED")
    print("="*80)
    print()
    print("Tmux session created with 5 panes:")
    print("  Pane 0 (top-left):     Gazebo + obstacles")
    print("  Pane 1 (top-right):    Navigation nodes")
    print("  Pane 2 (middle-left):  Odom position echo")
    print("  Pane 3 (middle-right): Odom_raw frequency")
    print("  Pane 4 (bottom-left):  Topic info (press Enter to refresh)")
    print()
    print("To attach to session:")
    print(f"  tmux attach -t {session_name}")
    print()
    print("To kill session:")
    print(f"  tmux kill-session -t {session_name}")
    print()
    print("WATCH FOR:")
    print("  1. Odom position suddenly going to (0, 0, 0)")
    print("  2. Odom_raw frequency dropping or stopping")
    print("  3. Multiple publishers on odom topic")
    print()

    # Attach to the tmux session
    print("Attaching to tmux session...")
    os.execvp('tmux', ['tmux', 'attach', '-t', session_name])


def main():
    parser = argparse.ArgumentParser(
        description='Run Intent-MPC benchmark trials with dynamic obstacles',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--num-trials', type=int, default=20,
                        help='Number of trials to run (default: 20)')
    parser.add_argument('--num-obstacles', type=int, default=200,
                        help='Number of obstacles (default: 200, matching DYNUS benchmark)')
    parser.add_argument('--dynamic-ratio', type=float, default=0.65,
                        help='Ratio of dynamic obstacles (default: 0.65, matching DYNUS benchmark)')
    parser.add_argument('--seed-start', type=int, default=0,
                        help='Starting seed value (default: 0)')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='Timeout per trial in seconds (default: 120)')
    parser.add_argument('--output-dir', type=str,
                        default='/root/ip-mpc_ws/src/Intent-MPC/data',
                        help='Output directory for results')
    parser.add_argument('--visualize', '--viz', '--rviz', action='store_true',
                        help='Launch with RViz visualization (default: headless)')
    parser.add_argument('--debug', action='store_true',
                        help='Launch in debug mode with tmux panes for monitoring')

    args = parser.parse_args()

    # Debug mode: run single trial with tmux monitoring
    if args.debug:
        run_debug_trial(args.num_obstacles, args.dynamic_ratio, args.seed_start)
        return

    # Handle visualization flag
    visualize = args.visualize

    # Create output directory with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = FilePath(args.output_dir) / f"benchmark_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=True)

    print("="*80)
    print("INTENT-MPC BENCHMARK RUNNER")
    print("="*80)
    print(f"Configuration:")
    print(f"  Trials: {args.num_trials}")
    print(f"  Obstacles: {args.num_obstacles}")
    print(f"  Dynamic ratio: {args.dynamic_ratio}")
    print(f"  Seed range: {args.seed_start} to {args.seed_start + args.num_trials - 1}")
    print(f"  Timeout: {args.timeout}s")
    print(f"  Output: {output_dir}")
    print(f"  Visualize: {visualize}")
    print("="*80)
    print()

    # Run trials
    metrics_list = []

    for i in range(args.num_trials):
        trial_id = i
        seed = args.seed_start + i

        print(f"\n{'='*80}")
        print(f"Starting Trial {trial_id+1}/{args.num_trials}")
        print(f"{'='*80}\n")

        try:
            metrics = run_trial(
                trial_id=trial_id,
                num_obstacles=args.num_obstacles,
                dynamic_ratio=args.dynamic_ratio,
                seed=seed,
                timeout=args.timeout,
                visualize=visualize,
                output_dir=output_dir
            )

            metrics_list.append(metrics)

            # Save intermediate results after each trial
            csv_path = output_dir / f"benchmark_intent_mpc_{timestamp}.csv"
            print(f"\nSaving intermediate results to: {csv_path}")
            save_metrics_csv(metrics_list, csv_path)

            print(f"\nTrial {trial_id} complete:")
            print(f"  Goal reached: {metrics.goal_reached}")
            print(f"  Flight time: {metrics.flight_travel_time:.2f}s")
            print(f"  Path length: {metrics.path_length:.2f}m")

        except Exception as e:
            print(f"\n{'!'*80}")
            print(f"ERROR in trial {trial_id}: {e}")
            print(f"{'!'*80}\n")
            import traceback
            traceback.print_exc()

            # Still save what we have so far
            if metrics_list:
                csv_path = output_dir / f"benchmark_intent_mpc_{timestamp}.csv"
                save_metrics_csv(metrics_list, csv_path)

        # Longer delay between trials to ensure clean shutdown
        if i < args.num_trials - 1:
            print(f"\nWaiting 8 seconds before next trial...\n")
            time.sleep(8)

            # Verify roscore is still healthy
            roscore_ok = subprocess.run(['pgrep', 'rosmaster'], capture_output=True).returncode == 0
            if not roscore_ok:
                print("WARNING: rosmaster died, restarting...")
                subprocess.Popen(['roscore'], preexec_fn=os.setsid,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                time.sleep(3)

    # Save final results
    print("\n" + "="*80)
    print("BENCHMARK COMPLETE")
    print("="*80)

    csv_path = output_dir / f"benchmark_intent_mpc_{timestamp}.csv"
    json_path = output_dir / f"benchmark_intent_mpc_{timestamp}.json"

    save_metrics_csv(metrics_list, csv_path)
    save_metrics_json(metrics_list, json_path)

    # Print summary
    if metrics_list:
        success_count = sum(1 for m in metrics_list if m.goal_reached)
        collision_count = sum(1 for m in metrics_list if m.collision)
        timeout_count = sum(1 for m in metrics_list if m.timeout_reached)

        print(f"\nSummary:")
        print(f"  Total trials: {len(metrics_list)}")
        print(f"  Successful: {success_count} ({100*success_count/len(metrics_list):.1f}%)")
        print(f"  Collisions: {collision_count} ({100*collision_count/len(metrics_list):.1f}%)")
        print(f"  Timeouts: {timeout_count} ({100*timeout_count/len(metrics_list):.1f}%)")
        print()
        print(f"Results saved to: {output_dir}")
        print()


if __name__ == '__main__':
    main()
