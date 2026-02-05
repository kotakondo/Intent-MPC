#!/bin/bash
# Debug obstacle detection pipeline

echo "========================================="
echo "Obstacle Detection Pipeline Debug"
echo "========================================="

source /root/ip-mpc_ws/devel/setup.bash

echo ""
echo "[1/5] Checking obstacle states publication..."
OBSTACLE_TOPIC=$(rostopic list 2>/dev/null | grep "model_states")
if [ -n "$OBSTACLE_TOPIC" ]; then
    echo "  ✓ Topic exists: $OBSTACLE_TOPIC"
    OBSTACLE_COUNT=$(timeout 2 rostopic echo /gazebo/model_states -n 1 2>/dev/null | grep -c "obstacle" || echo "0")
    echo "  ✓ Obstacles in topic: $OBSTACLE_COUNT"

    # Show first obstacle name
    FIRST_NAME=$(timeout 2 rostopic echo /gazebo/model_states/name -n 1 2>/dev/null | head -5)
    echo "  ✓ Sample names:"
    echo "$FIRST_NAME" | head -3
else
    echo "  ✗ No model_states topic found!"
fi

echo ""
echo "[2/5] Checking fake_detector output..."
FAKE_DET_TOPIC=$(rostopic list 2>/dev/null | grep "fake_detector")
if [ -n "$FAKE_DET_TOPIC" ]; then
    echo "  ✓ fake_detector topics exist"

    # Check if dynamic obstacles are being published
    timeout 3 rostopic echo /fake_detector/dynamic_obstacles -n 1 > /tmp/fake_det.txt 2>&1
    if [ -s /tmp/fake_det.txt ]; then
        DET_COUNT=$(cat /tmp/fake_det.txt | grep -c "id:" || echo "0")
        echo "  ✓ Detected obstacles: $DET_COUNT"
        if [ "$DET_COUNT" -eq "0" ]; then
            echo "  ⚠️  WARNING: No obstacles detected by fake_detector!"
        fi
    else
        echo "  ✗ fake_detector not publishing"
    fi
else
    echo "  ✗ No fake_detector topics found!"
fi

echo ""
echo "[3/5] Checking predictor output..."
PRED_TOPIC=$(rostopic list 2>/dev/null | grep "predictor")
if [ -n "$PRED_TOPIC" ]; then
    echo "  ✓ Predictor topics exist"
    timeout 3 rostopic echo /dynamic_predictor/predicted_trajectories -n 1 > /tmp/pred.txt 2>&1
    if [ -s /tmp/pred.txt ]; then
        PRED_COUNT=$(cat /tmp/pred.txt | grep -c "id:" || echo "0")
        echo "  ✓ Predicted trajectories: $PRED_COUNT"
    else
        echo "  ⚠️  Predictor not publishing yet"
    fi
else
    echo "  ✗ No predictor topics found!"
fi

echo ""
echo "[4/5] Checking MPC planner topics..."
rostopic list 2>/dev/null | grep -E "mpc_planner|mpcNavigation" | while read topic; do
    echo "  - $topic"
done

echo ""
echo "[5/5] Checking fake_detector parameter..."
rosparam get /target_obstacle 2>/dev/null || echo "  ✗ target_obstacle parameter not found"

echo ""
echo "========================================="
echo "Summary"
echo "========================================="
echo "If 'Detected obstacles: 0', the issue is:"
echo "  → fake_detector isn't matching obstacle names"
echo "  → Check obstacle name format in /gazebo/model_states"
echo ""
