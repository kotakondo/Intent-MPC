#!/bin/bash
# Test script to verify benchmark setup

echo "========================================="
echo "Testing Intent-MPC Benchmark Setup"
echo "========================================="
echo ""

cd /root/ip-mpc_ws
source devel/setup.bash

echo "1. Checking launch files..."
if [ -f "src/Intent-MPC/autonomous_flight/launch/intent_mpc_dynus_sim_headless.launch" ]; then
    echo "  ✓ intent_mpc_dynus_sim_headless.launch found"
else
    echo "  ✗ intent_mpc_dynus_sim_headless.launch NOT FOUND"
    exit 1
fi

echo ""
echo "2. Checking trajectory file..."
if [ -f "src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/ref_trajectory_dynus_benchmark.txt" ]; then
    echo "  ✓ ref_trajectory_dynus_benchmark.txt found"
    echo "  Trajectory points: $(wc -l < src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/ref_trajectory_dynus_benchmark.txt)"
else
    echo "  ✗ ref_trajectory_dynus_benchmark.txt NOT FOUND"
    exit 1
fi

echo ""
echo "3. Checking data directory..."
mkdir -p src/Intent-MPC/data
if [ -d "src/Intent-MPC/data" ]; then
    echo "  ✓ data directory created"
    echo "  Path: $(pwd)/src/Intent-MPC/data"
else
    echo "  ✗ Failed to create data directory"
    exit 1
fi

echo ""
echo "4. Testing quick launch (5 seconds)..."
echo "  Starting intent_mpc_dynus_sim_headless.launch..."
timeout 5s roslaunch autonomous_flight intent_mpc_dynus_sim_headless.launch num_obstacles:=10 dynamic_ratio:=0.65 seed:=0 > /tmp/launch_test.log 2>&1 &
LAUNCH_PID=$!

sleep 5

if ps -p $LAUNCH_PID > /dev/null; then
    echo "  ✓ Launch file started successfully"
    kill -9 $LAUNCH_PID 2>/dev/null
    killall -9 roslaunch rosmaster 2>/dev/null
else
    echo "  ✗ Launch file failed to start"
    echo "  Check log: /tmp/launch_test.log"
    exit 1
fi

echo ""
echo "5. Checking ROS topics (from log)..."
if grep -q "/CERLAB/quadcopter/odom" /tmp/launch_test.log; then
    echo "  ✓ Odometry topic detected"
else
    echo "  ⚠ Odometry topic not detected in log"
fi

echo ""
echo "========================================="
echo "Setup test complete!"
echo "========================================="
echo ""
echo "Next steps:"
echo "  1. Run a single trial test:"
echo "     python3 src/Intent-MPC/scripts/run_mpc_benchmark.py --num-trials 1 --num-obstacles 10 --timeout 30"
echo ""
echo "  2. Check output:"
echo "     ls -la src/Intent-MPC/data/"
echo ""
