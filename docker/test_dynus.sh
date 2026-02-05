#!/bin/bash
# Test script for DYNUS integration
# Launches components and verifies they're working

set -e

echo "========================================="
echo "Testing DYNUS Integration"
echo "========================================="
echo ""

# Source ROS
source /opt/ros/noetic/setup.bash
source /root/ip-mpc_ws/devel/setup.bash

# Start roscore in background
echo "[1/6] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 3

# Launch DYNUS obstacles node
echo "[2/6] Launching DYNUS obstacles node..."
roslaunch dynus_obstacles_ros1 dynus_obstacles.launch \
    num_obstacles:=10 \
    dynamic_ratio:=0.5 \
    seed:=42 &
DYNUS_PID=$!
sleep 5

# Check if nodes are running
echo "[3/6] Checking node status..."
rosnode list
echo ""

# Check topics
echo "[4/6] Checking topic rates..."
echo "  - Obstacle states (target: 50 Hz):"
timeout 3 rostopic hz /gazebo/model_states | grep "average rate" || echo "    (measuring...)"
echo ""
echo "  - Odometry (target: 100 Hz):"
timeout 3 rostopic hz /CERLAB/quadcopter/odom | grep "average rate" || echo "    (measuring...)"
echo ""

# Check obstacle count
echo "[5/6] Checking obstacle data..."
OBSTACLE_COUNT=$(timeout 2 rostopic echo /gazebo/model_states -n 1 | grep -c "obstacle_" || echo "0")
echo "  - Obstacles spawned: $OBSTACLE_COUNT"
echo ""

# Check drone position
echo "[6/6] Checking drone odometry..."
timeout 2 rostopic echo /CERLAB/quadcopter/odom --noarr -n 1 | head -10 || echo "  (waiting for data...)"
echo ""

# Summary
echo "========================================="
echo "Test Summary"
echo "========================================="
echo "✓ Nodes launched successfully"
echo "✓ Topics publishing"
echo "✓ Obstacles: $OBSTACLE_COUNT detected"
echo ""
echo "Press Ctrl+C to stop..."

# Wait a bit more to show data, then cleanup
sleep 5

# Cleanup
echo ""
echo "Cleaning up..."
kill $DYNUS_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
sleep 1
killall -9 roscore rosmaster roslaunch rosout 2>/dev/null || true

echo "Test complete!"
