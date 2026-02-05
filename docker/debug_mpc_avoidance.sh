#!/bin/bash
# Debug MPC obstacle avoidance pipeline

echo "========================================="
echo "MPC Obstacle Avoidance Debug"
echo "========================================="

source /root/ip-mpc_ws/devel/setup.bash

echo ""
echo "[1/6] Checking DYNUS obstacles published..."
timeout 2 rostopic hz /gazebo/model_states 2>&1 | grep "average rate" || echo "  ✗ Not publishing"

echo ""
echo "[2/6] Checking fake_detector output..."
timeout 2 rostopic hz /fake_detector/dynamic_obstacles 2>&1 | grep "average rate" || echo "  ✗ Not publishing"
DETECTED=$(timeout 2 rostopic echo /fake_detector/dynamic_obstacles -n 1 2>/dev/null | grep -c "id:" || echo "0")
echo "  Detected obstacles: $DETECTED"

echo ""
echo "[3/6] Checking predictor output..."
timeout 2 rostopic hz /dynamic_predictor/predicted_trajectories 2>&1 | grep "average rate" || echo "  ✗ Not publishing"
PREDICTED=$(timeout 2 rostopic echo /dynamic_predictor/predicted_trajectories -n 1 2>/dev/null | grep -c "id:" || echo "0")
echo "  Predicted trajectories: $PREDICTED"

echo ""
echo "[4/6] Checking MPC trajectory output..."
timeout 2 rostopic hz /mpcNavigation/mpc_trajectory 2>&1 | grep "average rate" || echo "  ✗ Not publishing"

echo ""
echo "[5/6] Checking MPC vs Reference deviation..."
echo "  (If MPC is avoiding, trajectories should differ)"
MPC_Y=$(timeout 1 rostopic echo /mpcNavigation/mpc_trajectory/poses[5]/pose/position/y -n 1 2>/dev/null || echo "N/A")
REF_Y=$(timeout 1 rostopic echo /mpcNavigation/input_trajectory/poses[5]/pose/position/y -n 1 2>/dev/null || echo "N/A")
echo "  MPC Y at waypoint 5: $MPC_Y"
echo "  REF Y at waypoint 5: $REF_Y"

echo ""
echo "[6/6] Checking for collision warnings..."
timeout 3 rostopic echo /rosout -n 20 2>/dev/null | grep -i "collision\|avoid" | tail -3

echo ""
echo "========================================="
echo "Summary"
echo "========================================="
if [ "$DETECTED" -gt "0" ] && [ "$PREDICTED" -gt "0" ]; then
    echo "✓ Detection/Prediction pipeline working"
    echo "  If MPC Y ≠ REF Y, avoidance is working"
    echo "  If MPC Y = REF Y = 0, MPC not avoiding!"
else
    echo "✗ Detection/Prediction pipeline BROKEN"
    echo "  Check if fake_detector_node and dynamic_predictor_fake_node are running"
fi
echo "========================================="
