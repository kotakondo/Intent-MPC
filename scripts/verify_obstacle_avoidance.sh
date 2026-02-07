#!/bin/bash
# Verification script to check if MPC is detecting and avoiding dynamic obstacles
# Run this while the simulation is running in another terminal

set -e

echo "========================================="
echo "Obstacle Avoidance Verification"
echo "========================================="
echo ""

source /root/ip-mpc_ws/devel/setup.bash

echo "[1/5] Checking if fake_detector is receiving obstacles..."
OBSTACLE_COUNT=$(timeout 2 rostopic echo /gazebo/model_states -n 1 2>/dev/null | grep -c "obstacle_" || echo "0")
if [ "$OBSTACLE_COUNT" -gt 0 ]; then
    echo "  ✓ Detected $OBSTACLE_COUNT obstacles on /gazebo/model_states"
else
    echo "  ✗ No obstacles detected"
    exit 1
fi
echo ""

echo "[2/5] Checking if predictor is generating predictions..."
if timeout 3 rostopic echo /dynamic_predictor/predicted_trajectories -n 1 > /dev/null 2>&1; then
    echo "  ✓ Predictor is publishing predicted trajectories"
    PRED_COUNT=$(timeout 2 rostopic echo /dynamic_predictor/predicted_trajectories -n 1 2>/dev/null | grep -c "id:" || echo "0")
    echo "  ✓ Number of predicted obstacles: $PRED_COUNT"
else
    echo "  ⚠ Predictor not publishing yet (may need more time)"
fi
echo ""

echo "[3/5] Checking MPC trajectory generation..."
if timeout 3 rostopic echo /mpcNavigation/mpc_trajectory -n 1 > /dev/null 2>&1; then
    echo "  ✓ MPC is generating trajectories"
    TRAJ_POINTS=$(timeout 2 rostopic echo /mpcNavigation/mpc_trajectory -n 1 2>/dev/null | grep -c "x:" || echo "0")
    echo "  ✓ Trajectory points in plan: $TRAJ_POINTS"
else
    echo "  ⚠ MPC trajectory not publishing yet"
fi
echo ""

echo "[4/5] Checking if drone is deviating from straight line..."
echo "  Sampling drone position over 5 seconds..."
POS_Y1=$(timeout 2 rostopic echo /CERLAB/quadcopter/odom/pose/pose/position/y -n 1 2>/dev/null || echo "0")
sleep 2
POS_Y2=$(timeout 2 rostopic echo /CERLAB/quadcopter/odom/pose/pose/position/y -n 1 2>/dev/null || echo "0")
sleep 2
POS_Y3=$(timeout 2 rostopic echo /CERLAB/quadcopter/odom/pose/pose/position/y -n 1 2>/dev/null || echo "0")

echo "  Y positions: t=0s: $POS_Y1, t=2s: $POS_Y2, t=4s: $POS_Y3"

# Check if Y position changes (deviation from straight line y=0)
if (( $(echo "$POS_Y1 != 0 || $POS_Y2 != 0 || $POS_Y3 != 0" | bc -l 2>/dev/null || echo "0") )); then
    echo "  ✓ Drone is deviating from straight line (avoiding obstacles)"
else
    echo "  ⚠ Drone staying on straight line (may be no obstacles nearby yet)"
fi
echo ""

echo "[5/5] Obstacle proximity check..."
echo "  Checking distance to nearest obstacle..."
# This is a simplified check - in real scenario you'd compute actual distances
timeout 2 rostopic echo /gazebo/model_states -n 1 2>/dev/null | grep -A 5 "obstacle_0" | head -8
echo ""

echo "========================================="
echo "Verification Summary"
echo "========================================="
echo "If you see:"
echo "  - ✓ Obstacles detected"
echo "  - ✓ Predictor running"
echo "  - ✓ MPC generating trajectories"
echo "  - ✓ Drone deviating from straight line"
echo ""
echo "Then obstacle avoidance is working correctly!"
echo ""
echo "Monitor RViz to visually confirm:"
echo "  - Red cubes moving in trefoil patterns (dynamic obstacles)"
echo "  - Blue cubes stationary (static obstacles)"
echo "  - Green path curving around obstacles"
echo "========================================="
