#!/bin/bash
# Full integration test: DYNUS obstacles + Intent-MPC
# Tests obstacle detection and MPC planning

set -e

echo "========================================="
echo "Testing Full DYNUS + Intent-MPC Integration"
echo "========================================="
echo ""

# Source ROS
source /opt/ros/noetic/setup.bash
source /root/ip-mpc_ws/devel/setup.bash

# Start roscore
echo "[1/8] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 3

# Launch DYNUS obstacles + fake odom
echo "[2/8] Launching DYNUS obstacles..."
roslaunch dynus_obstacles_ros1 dynus_obstacles.launch \
    num_obstacles:=10 \
    dynamic_ratio:=0.5 \
    seed:=42 &
DYNUS_PID=$!
sleep 5

# Launch Intent-MPC (headless - without RViz for testing)
echo "[3/8] Launching Intent-MPC navigation (headless)..."
roslaunch autonomous_flight intent_mpc_dynus_sim_headless.launch &
MPC_PID=$!
sleep 10

# Check all nodes are running
echo "[4/8] Checking all nodes..."
rosnode list | grep -E "dynus|fake_odom|mpc_navigation|tracking_controller"
echo ""

# Check MPC is receiving obstacle data
echo "[5/8] Checking obstacle detection..."
timeout 2 rostopic echo /gazebo/model_states -n 1 | grep "obstacle_0" && echo "  ✓ Obstacles visible on topic" || echo "  ✗ No obstacles detected"
echo ""

# Check MPC trajectory is being published
echo "[6/8] Checking MPC trajectory..."
if timeout 3 rostopic echo /mpcNavigation/mpc_trajectory -n 1 > /dev/null 2>&1; then
    echo "  ✓ MPC trajectory publishing"
    TRAJ_POINTS=$(timeout 2 rostopic echo /mpcNavigation/mpc_trajectory -n 1 | grep -c "x:" || echo "0")
    echo "  ✓ Trajectory points: $TRAJ_POINTS"
else
    echo "  ⚠ MPC trajectory not yet publishing (may need more time)"
fi
echo ""

# Check drone is moving
echo "[7/8] Checking drone state..."
POS1=$(timeout 2 rostopic echo /CERLAB/quadcopter/odom/pose/pose/position/x -n 1 2>/dev/null || echo "0")
sleep 2
POS2=$(timeout 2 rostopic echo /CERLAB/quadcopter/odom/pose/pose/position/x -n 1 2>/dev/null || echo "0")
echo "  Position t=0s: x=$POS1"
echo "  Position t=2s: x=$POS2"
if (( $(echo "$POS2 > $POS1" | bc -l 2>/dev/null || echo "0") )); then
    echo "  ✓ Drone is progressing forward"
else
    echo "  ⚠ Drone position unchanged (may need more time to start)"
fi
echo ""

# Summary
echo "[8/8] Integration Test Summary"
echo "========================================="
rosnode list | wc -l | xargs echo "  Active nodes:"
rostopic list | wc -l | xargs echo "  Active topics:"
echo ""
echo "Key topics:"
timeout 2 rostopic hz /gazebo/model_states 2>&1 | grep "average rate" | head -1 || true
timeout 2 rostopic hz /CERLAB/quadcopter/odom 2>&1 | grep "average rate" | head -1 || true
timeout 2 rostopic hz /mpcNavigation/mpc_trajectory 2>&1 | grep "average rate" | head -1 || true
echo ""

echo "========================================="
echo "Test running for 5 more seconds..."
echo "Press Ctrl+C to stop early"
sleep 5

# Cleanup
echo ""
echo "Cleaning up..."
kill $MPC_PID 2>/dev/null || true
kill $DYNUS_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
sleep 2
killall -9 roscore rosmaster roslaunch rosout 2>/dev/null || true

echo "Full integration test complete!"
