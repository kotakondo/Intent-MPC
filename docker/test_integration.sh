#!/bin/bash
# Comprehensive integration test
set -e

echo "====================================="
echo "Integration Test - Dynamic Obstacles"
echo "====================================="

cd /root/ip-mpc_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Start roscore
echo "[1/5] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 3

# Launch system
echo "[2/5] Launching system..."
roslaunch autonomous_flight intent_mpc_dynus_sim_headless.launch \
    num_obstacles:=10 \
    dynamic_ratio:=0.7 \
    seed:=42 &
LAUNCH_PID=$!

# Wait for initialization
echo "[3/5] Waiting for initialization (15s)..."
sleep 15

# Check nodes
echo "[4/5] Checking nodes..."
NODES=$(rosnode list 2>&1 | wc -l)
echo "  Active nodes: $NODES"
rosnode list 2>&1 | grep -E "dynus|fake_odom|mpc|controller" || echo "  No matching nodes found"

# Check if mpc_navigation crashed
if ! rosnode list 2>&1 | grep -q "mpc_navigation_node"; then
    echo "  ✗ ERROR: mpc_navigation_node not running!"
    echo ""
    echo "  Checking logs..."
    find /root/.ros/log -name "mpc_navigation_node*.log" -exec tail -50 {} \;
else
    echo "  ✓ mpc_navigation_node is running"
fi

# Check topics
echo "[5/5] Checking topics..."
rostopic list 2>&1 | grep -c "/mpcNavigation/mpc_trajectory" && echo "  ✓ MPC trajectory topic exists" || echo "  ✗ MPC trajectory missing"
rostopic list 2>&1 | grep -c "/gazebo/model_states" && echo "  ✓ Obstacle states topic exists" || echo "  ✗ Obstacle states missing"
rostopic list 2>&1 | grep -c "/fake_detector/dynamic_obstacles" && echo "  ✓ Detected obstacles topic exists" || echo "  ✗ Detected obstacles missing"

# Cleanup
echo ""
echo "Cleaning up..."
kill $LAUNCH_PID $ROSCORE_PID 2>/dev/null || true
sleep 2
killall -9 roscore rosmaster roslaunch rosout 2>/dev/null || true

echo "Test complete!"
