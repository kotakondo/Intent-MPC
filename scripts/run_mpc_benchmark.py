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
from std_msgs.msg import String, Float64
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

    # Constraint violations (DYNUS benchmark limits) - Linf norm (per-axis)
    vel_limit: float = 5.0  # m/s (DYNUS: v_max)
    acc_limit: float = 20.0  # m/s² (DYNUS: a_max)
    jerk_limit: float = 100.0  # m/s³ (DYNUS: j_max)

    # Per-timestep Linf violation counts (DYNUS methodology):
    # A timestep is a violation if ANY axis exceeds the limit + tolerance.
    vel_violation_count: int = 0
    acc_violation_count: int = 0
    jerk_violation_count: int = 0
    vel_total_samples: int = 0
    acc_total_samples: int = 0
    jerk_total_samples: int = 0

    # Per-axis violation counts (kept for diagnostics)
    vel_violation_count_x: int = 0
    vel_violation_count_y: int = 0
    vel_violation_count_z: int = 0
    acc_violation_count_x: int = 0
    acc_violation_count_y: int = 0
    acc_violation_count_z: int = 0
    jerk_violation_count_x: int = 0
    jerk_violation_count_y: int = 0
    jerk_violation_count_z: int = 0

    # Per-axis max values observed
    vel_max_x: float = 0.0
    vel_max_y: float = 0.0
    vel_max_z: float = 0.0
    acc_max_x: float = 0.0
    acc_max_y: float = 0.0
    acc_max_z: float = 0.0
    jerk_max_x: float = 0.0
    jerk_max_y: float = 0.0
    jerk_max_z: float = 0.0

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

    # MPC weight configuration
    acceleration_weight: float = 10.0
    max_vel: float = 5.0

    # Difficulty configuration
    difficulty: str = ""  # easy, medium, hard

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

        # COMMANDED trajectory data from target_state (for vel/acc/jerk metrics)
        self.target_data: List[Tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []  # (time, pos, vel, acc)
        self.last_target_acc = None
        self.last_target_time = None

        # Goal tracking (DYNUS benchmark: (0,0,2) -> (105,0,2))
        self.goal_position = np.array([105.0, 0.0, 2.0])
        self.start_position = np.array([0.0, 0.0, 2.0])
        self.goal_threshold = 0.5  # meters
        self.start_threshold = 10.0  # meters - must be near start before monitoring begins
        self.waiting_for_start = True  # Don't start monitoring until drone is near start position
        self.monitor_created_time = time.time()  # Wall clock time for timeout
        self.odom_callback_count = 0  # Debug: count callbacks

        # MPC compute time tracking
        self.mpc_compute_times: List[float] = []  # seconds per solve

        # Obstacle tracking for collision detection
        self.obstacle_positions = {}  # {name: position}
        self.obstacle_sizes = {}  # {name: (x_size, y_size, z_size)}

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

        # MPC compute time subscriber
        self.mpc_compute_time_sub = rospy.Subscriber('/mpcNavigation/mpc_compute_time', Float64, self.mpc_compute_time_callback)
        rospy.loginfo("Subscribed to /mpcNavigation/mpc_compute_time for solver timing")

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

        # start_time is set by the first target_state callback (first command message).
        # If we haven't received a command yet, record odom at t=0 for path tracking.
        if self.start_time is None:
            current_time = 0.0
        else:
            current_time = rospy.Time.now().to_sec() - self.start_time

        # Store actual position for path length calculation
        self.odom_data.append((current_time, pos))

        self.last_position = pos
        self.last_time = current_time

        # Extract velocity from odometry for goal reached check
        vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        vel_magnitude = np.linalg.norm(vel)

        # Check if goal reached (distance + velocity threshold only, no minimum flight time)
        goal_distance = np.linalg.norm(pos - self.goal_position)
        vel_threshold = 0.1  # m/s - drone must be nearly stopped to count as "goal reached"
        if goal_distance < self.goal_threshold and vel_magnitude < vel_threshold:
            if not self.is_complete:
                self.is_complete = True
                self.completion_reason = "goal_reached"
                self.metrics.goal_reached = True
                self.metrics.flight_travel_time = current_time
                rospy.loginfo(f"Trial {self.metrics.trial_id}: *** GOAL REACHED *** in {current_time:.2f}s (distance: {goal_distance:.2f}m, vel: {vel_magnitude:.3f}m/s)")


    def target_callback(self, msg):
        """Collect COMMANDED pos/vel/acc from target_state (like DYNUS /NX01/goal)"""
        # Travel time starts at the FIRST command message (first target_state callback)
        if self.start_time is None:
            self.start_time = rospy.Time.now().to_sec()
            rospy.loginfo(f"Trial {self.metrics.trial_id}: Travel time started at first target_state command (t={self.start_time:.3f})")

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
                # Use index as key since all obstacles of the same type share the same name
                # (e.g., all dynamic cubes are "obstacle_d080_080_080")
                obs_key = i
                self.obstacle_positions[obs_key] = np.array([pos.x, pos.y, pos.z])

                # Parse obstacle size from name (e.g., "obstacle_d080_080_080")
                if obs_key not in self.obstacle_sizes:
                    size = self.parse_obstacle_size(name)
                    if size is not None:
                        self.obstacle_sizes[obs_key] = size

        # Log obstacle count periodically (first time and every 500 callbacks)
        if not hasattr(self, '_model_states_callback_count'):
            self._model_states_callback_count = 0
        self._model_states_callback_count += 1
        if self._model_states_callback_count == 1:
            print(f"*** DYNUS MODEL_STATES CALLBACK RECEIVED! Trial {self.metrics.trial_id}: tracking {len(self.obstacle_positions)} obstacles ***", flush=True)
            rospy.loginfo(f"Trial {self.metrics.trial_id}: model_states callback #{self._model_states_callback_count}, tracking {len(self.obstacle_positions)} obstacles")
        elif self._model_states_callback_count % 500 == 0:
            rospy.loginfo(f"Trial {self.metrics.trial_id}: model_states callback #{self._model_states_callback_count}, tracking {len(self.obstacle_positions)} obstacles")

        # Check for collisions if we have odometry data
        if self.last_position is not None:
            self.check_collisions()

    def mpc_compute_time_callback(self, msg: Float64):
        """Collect MPC solver computation time per solve"""
        if self.start_time is not None:
            self.mpc_compute_times.append(msg.data)

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
        """Check for collisions with all obstacles.

        Uses point-to-AABB Euclidean distance: distance from drone center
        to the closest point on the obstacle box surface.
        """
        if self.last_position is None:
            return

        for obs_key, obs_pos in self.obstacle_positions.items():
            obs_size = self.obstacle_sizes.get(obs_key, (0.8, 0.8, 0.8))
            obs_half = (obs_size[0] / 2.0, obs_size[1] / 2.0, obs_size[2] / 2.0)

            # Per-axis gap from drone center to obstacle surface (0 if overlapping on that axis)
            dx = max(0.0, abs(self.last_position[0] - obs_pos[0]) - obs_half[0])
            dy = max(0.0, abs(self.last_position[1] - obs_pos[1]) - obs_half[1])
            dz = max(0.0, abs(self.last_position[2] - obs_pos[2]) - obs_half[2])

            # Euclidean distance from drone center to closest point on box surface
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            self.metrics.min_distance_to_obstacles = min(self.metrics.min_distance_to_obstacles, distance)

            # Collision: drone center is inside the box (distance == 0)
            if distance == 0.0:
                self.metrics.collision_count += 1
                pen_x = obs_half[0] - abs(self.last_position[0] - obs_pos[0])
                pen_y = obs_half[1] - abs(self.last_position[1] - obs_pos[1])
                pen_z = obs_half[2] - abs(self.last_position[2] - obs_pos[2])
                penetration = min(pen_x, pen_y, pen_z)
                self.metrics.collision_penetration_max = max(self.metrics.collision_penetration_max, penetration)

                if not self.metrics.collision:
                    self.metrics.collision = True
                    rospy.logwarn(f"Trial {self.metrics.trial_id}: Collision detected with obstacle idx={obs_key}")

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

            rospy.loginfo(f"Trial {self.metrics.trial_id}: Commanded Velocity - avg: {self.metrics.avg_velocity:.2f} m/s, max: {self.metrics.max_velocity:.2f} m/s (limit: {self.metrics.vel_limit} m/s, Linf)")

            # Velocity violations - Linf norm (DYNUS: any axis exceeds → 1 violation)
            self.metrics.vel_total_samples = len(velocities)
            for vel in velocities:
                # Track max per axis
                self.metrics.vel_max_x = max(self.metrics.vel_max_x, abs(vel[0]))
                self.metrics.vel_max_y = max(self.metrics.vel_max_y, abs(vel[1]))
                self.metrics.vel_max_z = max(self.metrics.vel_max_z, abs(vel[2]))
                # Per-axis counts (diagnostics)
                vx_viol = abs(vel[0]) > self.metrics.vel_limit + 1e-3
                vy_viol = abs(vel[1]) > self.metrics.vel_limit + 1e-3
                vz_viol = abs(vel[2]) > self.metrics.vel_limit + 1e-3
                if vx_viol:
                    self.metrics.vel_violation_count_x += 1
                if vy_viol:
                    self.metrics.vel_violation_count_y += 1
                if vz_viol:
                    self.metrics.vel_violation_count_z += 1
                # Per-timestep Linf count (any axis)
                if vx_viol or vy_viol or vz_viol:
                    self.metrics.vel_violation_count += 1

            if self.metrics.vel_violation_count > 0:
                vel_rate = self.metrics.vel_violation_count / self.metrics.vel_total_samples * 100
                rospy.logwarn(f"Trial {self.metrics.trial_id}: VELOCITY VIOLATIONS (Linf): {self.metrics.vel_violation_count}/{self.metrics.vel_total_samples} ({vel_rate:.1f}%) - x={self.metrics.vel_violation_count_x} (max {self.metrics.vel_max_x:.2f}), y={self.metrics.vel_violation_count_y} (max {self.metrics.vel_max_y:.2f}), z={self.metrics.vel_violation_count_z} (max {self.metrics.vel_max_z:.2f}) m/s")

            # Acceleration metrics from commanded trajectory
            acc_magnitudes = np.linalg.norm(accelerations, axis=1)
            self.metrics.avg_acceleration = float(np.mean(acc_magnitudes))
            self.metrics.max_acceleration = float(np.max(acc_magnitudes))

            rospy.loginfo(f"Trial {self.metrics.trial_id}: Commanded Acceleration - avg: {self.metrics.avg_acceleration:.2f} m/s², max: {self.metrics.max_acceleration:.2f} m/s² (limit: {self.metrics.acc_limit} m/s², Linf)")

            # Acceleration violations - Linf norm (DYNUS: any axis exceeds → 1 violation)
            self.metrics.acc_total_samples = len(accelerations)
            for acc in accelerations:
                # Track max per axis
                self.metrics.acc_max_x = max(self.metrics.acc_max_x, abs(acc[0]))
                self.metrics.acc_max_y = max(self.metrics.acc_max_y, abs(acc[1]))
                self.metrics.acc_max_z = max(self.metrics.acc_max_z, abs(acc[2]))
                # Per-axis counts (diagnostics)
                ax_viol = abs(acc[0]) > self.metrics.acc_limit + 1e-3
                ay_viol = abs(acc[1]) > self.metrics.acc_limit + 1e-3
                az_viol = abs(acc[2]) > self.metrics.acc_limit + 1e-3
                if ax_viol:
                    self.metrics.acc_violation_count_x += 1
                if ay_viol:
                    self.metrics.acc_violation_count_y += 1
                if az_viol:
                    self.metrics.acc_violation_count_z += 1
                # Per-timestep Linf count (any axis)
                if ax_viol or ay_viol or az_viol:
                    self.metrics.acc_violation_count += 1

            if self.metrics.acc_violation_count > 0:
                acc_rate = self.metrics.acc_violation_count / self.metrics.acc_total_samples * 100
                rospy.logwarn(f"Trial {self.metrics.trial_id}: ACCELERATION VIOLATIONS (Linf): {self.metrics.acc_violation_count}/{self.metrics.acc_total_samples} ({acc_rate:.1f}%) - x={self.metrics.acc_violation_count_x} (max {self.metrics.acc_max_x:.2f}), y={self.metrics.acc_violation_count_y} (max {self.metrics.acc_max_y:.2f}), z={self.metrics.acc_violation_count_z} (max {self.metrics.acc_max_z:.2f}) m/s²")

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

                    rospy.loginfo(f"Trial {self.metrics.trial_id}: Jerk - RMS: {self.metrics.jerk_rms:.2f} m/s³, Integral: {self.metrics.jerk_integral:.2f} (limit: {self.metrics.jerk_limit} m/s³, Linf)")

                    # Jerk violations - Linf norm (DYNUS: any axis exceeds → 1 violation)
                    self.metrics.jerk_total_samples = len(jerk_vectors)
                    for jerk in jerk_vectors:
                        # Track max per axis
                        self.metrics.jerk_max_x = max(self.metrics.jerk_max_x, abs(jerk[0]))
                        self.metrics.jerk_max_y = max(self.metrics.jerk_max_y, abs(jerk[1]))
                        self.metrics.jerk_max_z = max(self.metrics.jerk_max_z, abs(jerk[2]))
                        # Per-axis counts (diagnostics)
                        jx_viol = abs(jerk[0]) > self.metrics.jerk_limit + 1e-3
                        jy_viol = abs(jerk[1]) > self.metrics.jerk_limit + 1e-3
                        jz_viol = abs(jerk[2]) > self.metrics.jerk_limit + 1e-3
                        if jx_viol:
                            self.metrics.jerk_violation_count_x += 1
                        if jy_viol:
                            self.metrics.jerk_violation_count_y += 1
                        if jz_viol:
                            self.metrics.jerk_violation_count_z += 1
                        # Per-timestep Linf count (any axis)
                        if jx_viol or jy_viol or jz_viol:
                            self.metrics.jerk_violation_count += 1

                    if self.metrics.jerk_violation_count > 0:
                        jerk_rate = self.metrics.jerk_violation_count / self.metrics.jerk_total_samples * 100
                        rospy.logwarn(f"Trial {self.metrics.trial_id}: JERK VIOLATIONS (Linf): {self.metrics.jerk_violation_count}/{self.metrics.jerk_total_samples} ({jerk_rate:.1f}%) - x={self.metrics.jerk_violation_count_x} (max {self.metrics.jerk_max_x:.2f}), y={self.metrics.jerk_violation_count_y} (max {self.metrics.jerk_max_y:.2f}), z={self.metrics.jerk_violation_count_z} (max {self.metrics.jerk_max_z:.2f}) m/s³")

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

        # MPC compute time metrics
        if len(self.mpc_compute_times) > 0:
            times = np.array(self.mpc_compute_times)
            self.metrics.mpc_compute_time_avg = float(np.mean(times))
            self.metrics.mpc_compute_time_max = float(np.max(times))
            self.metrics.mpc_compute_time_std = float(np.std(times))
            self.metrics.mpc_solve_count = len(self.mpc_compute_times)
            rospy.loginfo(f"Trial {self.metrics.trial_id}: MPC compute time - avg: {self.metrics.mpc_compute_time_avg*1000:.2f}ms, max: {self.metrics.mpc_compute_time_max*1000:.2f}ms, std: {self.metrics.mpc_compute_time_std*1000:.2f}ms, solves: {self.metrics.mpc_solve_count}")
        else:
            rospy.logwarn(f"Trial {self.metrics.trial_id}: No MPC compute time data collected - check /mpcNavigation/mpc_compute_time topic")

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


def launch_simulation_and_navigation(num_obstacles: int, dynamic_ratio: float, seed: int, visualize: bool = False, acceleration_weight: float = 10.0, max_vel: float = 5.0, max_acc: float = 20.0, output_dir: FilePath = None, bag_path: FilePath = None, bag_topics: List[str] = None):
    """Launch Intent-MPC with DYNUS obstacles using split launch (like working run-dynamic-gazebo)

    This mimics the exact tmuxp approach from dynus_sim.yml:
    1. Start roscore first (separate process)
    2. Wait 3 seconds
    3. Launch Gazebo + obstacles (start_dynus.launch) with gui:=false
    4. Wait 5 more seconds (8 total from start)
    5. Send takeoff command
    5c. Start rosbag recording (BEFORE navigation launch to capture early trajectory)
    5d. Wait 10s for rosbag to be fully ready
    6. Launch navigation (intent_mpc_dynus_nav.launch)

    Args:
        bag_path: Path for rosbag file. If provided, starts recording before navigation launch.
        bag_topics: List of ROS topics to record. Required if bag_path is provided.
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

    # Write subprocess output to log files (NOT PIPE) to prevent pipe buffer deadlock
    log_dir = output_dir if output_dir else FilePath('/tmp')
    log_dir.mkdir(parents=True, exist_ok=True)
    gazebo_log = open(log_dir / 'gazebo.log', 'w')
    gazebo_proc = subprocess.Popen(gazebo_cmd, preexec_fn=os.setsid,
                                   stdout=gazebo_log, stderr=subprocess.STDOUT)

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

    # STEP 5c: Start rosbag recording BEFORE navigation launch
    # This ensures early trajectory data is captured from the very start of navigation.
    rosbag_proc = None
    if bag_path is not None and bag_topics is not None:
        print("Step 5c: Starting rosbag recording BEFORE navigation launch...", flush=True)
        rosbag_proc = start_rosbag_recording(bag_path, bag_topics)
        print("Step 5d: Waiting 10s for rosbag to be fully ready...", flush=True)
        time.sleep(10)
    else:
        print("Step 5c: Skipping rosbag recording (no bag_path provided)", flush=True)

    # STEP 6: Launch navigation nodes (like tmuxp pane 3 second command)
    nav_cmd = [
        'roslaunch',
        'autonomous_flight',
        'intent_mpc_dynus_nav.launch',
        f'acceleration_weight:={acceleration_weight}',
        f'max_vel:={max_vel}',
        f'max_acc:={max_acc}'
    ]

    print(f"Step 6: Launching navigation (max_vel={max_vel}, max_acc={max_acc}, acceleration_weight={acceleration_weight})...", flush=True)
    print(f"  Command: {' '.join(nav_cmd)}", flush=True)

    nav_log = open(log_dir / 'navigation.log', 'w')
    nav_proc = subprocess.Popen(nav_cmd, preexec_fn=os.setsid,
                                stdout=nav_log, stderr=subprocess.STDOUT)

    # Store nav_proc and rosbag_proc so we can kill them later
    gazebo_proc.nav_proc = nav_proc
    gazebo_proc.rosbag_proc = rosbag_proc

    # Wait for navigation to initialize
    print("Step 7: Waiting for navigation to initialize (5s)...", flush=True)
    time.sleep(5)

    print(f"All systems launched successfully", flush=True)
    print(f"  Navigation log: {log_dir / 'navigation.log'}", flush=True)
    print(f"  Gazebo log: {log_dir / 'gazebo.log'}", flush=True)
    return gazebo_proc


def run_trial(trial_id: int, num_obstacles: int, dynamic_ratio: float, seed: int,
              timeout: float = 100.0, visualize: bool = False, output_dir: FilePath = None,
              acceleration_weight: float = 10.0, max_vel: float = 5.0, max_acc: float = 20.0,
              difficulty: str = "") -> BenchmarkMetrics:
    """Run a single benchmark trial"""

    print("="*80, flush=True)
    print(f"TRIAL {trial_id}: seed={seed}, obstacles={num_obstacles}, ratio={dynamic_ratio}", flush=True)
    print("="*80, flush=True)

    # Set default output directory early (needed for log files)
    if output_dir is None:
        output_dir = FilePath('/root/ip-mpc_ws/src/Intent-MPC/data')
    output_dir.mkdir(parents=True, exist_ok=True)

    # PRE-TRIAL CLEANUP: Kill any lingering processes from previous trials
    # Keep roscore alive - rospy can only init_node once per Python process
    if trial_id > 0:
        print(f"Trial {trial_id}: Pre-trial cleanup (keeping roscore)...", flush=True)
        kill_all_ros_processes(keep_roscore=True)
    else:
        print(f"Trial {trial_id}: First trial, skipping pre-cleanup", flush=True)

    # Prepare rosbag recording parameters (bag starts INSIDE launch, before navigation)
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

    # FIRST: Launch simulation and navigation (this starts roscore)
    # Rosbag recording starts INSIDE this function, BEFORE navigation launch,
    # so that early trajectory data is captured from the very beginning.
    sim_nav_proc = launch_simulation_and_navigation(num_obstacles, dynamic_ratio, seed, visualize, acceleration_weight, max_vel, max_acc, output_dir, bag_path=bag_path, bag_topics=topics_to_record)

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
    monitor.metrics.acceleration_weight = acceleration_weight
    monitor.metrics.max_vel = max_vel
    monitor.metrics.difficulty = difficulty

    # Rosbag process was started inside launch_simulation_and_navigation()
    # Retrieve the handle for cleanup later
    rosbag_proc = getattr(sim_nav_proc, 'rosbag_proc', None)

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
        if rosbag_proc is not None:
            try:
                rosbag_proc.send_signal(signal.SIGINT)  # Graceful stop
                rosbag_proc.wait(timeout=5)
                print(f"Trial {trial_id}: Rosbag saved to {bag_path}")
            except:
                rosbag_proc.kill()
        else:
            print(f"Trial {trial_id}: No rosbag process to stop")

        # Print tail of navigation log (MPC diagnostics)
        nav_log_path = output_dir / 'navigation.log'
        if nav_log_path.exists():
            print(f"\n--- Navigation log (last 30 lines) ---", flush=True)
            try:
                with open(nav_log_path, 'r') as f:
                    lines = f.readlines()
                    for line in lines[-30:]:
                        print(f"  {line.rstrip()}", flush=True)
            except Exception as e:
                print(f"  (could not read log: {e})", flush=True)
            print(f"--- End navigation log ---\n", flush=True)

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
            monitor.mpc_compute_time_sub.unregister()
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
    parser.add_argument('--timeout', type=float, default=100.0,
                        help='Timeout per trial in seconds (default: 100)')
    parser.add_argument('--output-dir', type=str,
                        default='/root/ip-mpc_ws/src/Intent-MPC/data',
                        help='Output directory for results')
    parser.add_argument('--acceleration-weight', type=float, default=10.0,
                        help='MPC acceleration weight (default: 10.0)')
    parser.add_argument('--max-vel', type=float, default=5.0,
                        help='Maximum velocity in m/s (default: 5.0)')
    parser.add_argument('--max-acc', type=float, default=20.0,
                        help='Maximum acceleration in m/s^2 (default: 20.0)')
    parser.add_argument('--difficulty', type=str, default=None,
                        choices=['easy', 'medium', 'hard'],
                        help='Difficulty level: easy (50 obs), medium (100 obs), hard (200 obs). Overrides --num-obstacles.')
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

    # Difficulty mapping: override num_obstacles if difficulty is set
    difficulty_map = {'easy': 50, 'medium': 100, 'hard': 200}
    difficulty = args.difficulty
    if difficulty is not None:
        args.num_obstacles = difficulty_map[difficulty]
    else:
        # Infer difficulty from num_obstacles
        reverse_map = {v: k for k, v in difficulty_map.items()}
        difficulty = reverse_map.get(args.num_obstacles, f'obs{args.num_obstacles}')

    # Create output directory with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = FilePath(args.output_dir) / f"{difficulty}_benchmark_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=True)

    print("="*80)
    print("INTENT-MPC BENCHMARK RUNNER")
    print("="*80)
    print(f"Configuration:")
    print(f"  Difficulty: {difficulty} ({args.num_obstacles} obstacles)")
    print(f"  Trials: {args.num_trials}")
    print(f"  Dynamic ratio: {args.dynamic_ratio}")
    print(f"  Seed range: {args.seed_start} to {args.seed_start + args.num_trials - 1}")
    print(f"  Timeout: {args.timeout}s")
    print(f"  Max velocity: {args.max_vel} m/s")
    print(f"  Max acceleration: {args.max_acc} m/s^2")
    print(f"  Acceleration weight: {args.acceleration_weight}")
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
                output_dir=output_dir,
                acceleration_weight=args.acceleration_weight,
                max_vel=args.max_vel,
                max_acc=args.max_acc,
                difficulty=difficulty,
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
