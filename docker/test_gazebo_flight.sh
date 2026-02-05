#!/bin/bash
# Test script to debug Gazebo flight and obstacle avoidance

echo "========================================="
echo "Gazebo Flight Debug Script"
echo "========================================="

source /root/ip-mpc_ws/devel/setup.bash

# Wait for Gazebo to be ready
sleep 2

echo ""
echo "[1/5] Checking drone spawn..."
rostopic echo /CERLAB/quadcopter/pose -n 1 --noarr

echo ""
echo "[2/5] Checking if Gazebo plugin is publishing odom..."
timeout 2 rostopic hz /CERLAB/quadcopter/odom_raw

echo ""
echo "[3/5] Checking drone state (should show position on ground)..."
timeout 1 rostopic echo /CERLAB/quadcopter/odom -n 1 --noarr

echo ""
echo "[4/5] Sending manual takeoff command..."
rostopic pub -1 /CERLAB/quadcopter/takeoff std_msgs/Empty

echo "Waiting 2 seconds for state transition..."
sleep 2

echo ""
echo "[5/5] Checking if drone altitude changed..."
timeout 1 rostopic echo /CERLAB/quadcopter/odom -n 1 --noarr

echo ""
echo "========================================="
echo "Now sending a high position target to test..."
rostopic pub -1 /CERLAB/quadcopter/setpoint_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 3.0}}}"

echo "Wait 5 seconds and check if drone moves up..."
sleep 5

echo ""
echo "Final position check:"
timeout 1 rostopic echo /CERLAB/quadcopter/odom -n 1 --noarr

echo ""
echo "========================================="
echo "If z position increased, takeoff works!"
echo "If not, check Gazebo plugin state machine."
echo "========================================="
