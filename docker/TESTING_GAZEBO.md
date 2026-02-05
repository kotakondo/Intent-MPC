# Testing Gazebo Flight Without Full Docker Rebuild

Since the workspace is mounted with `-v` flag in `run-dev` mode, you can edit files on host and rebuild inside container.

## Quick Development Workflow

### 1. Start Container in Dev Mode
```bash
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker
make run-dev
```

### 2. Inside Container - Rebuild After Editing Launch Files
```bash
cd /root/ip-mpc_ws
catkin_make  # Only needed if you changed C++ code
source devel/setup.bash
```

### 3. Test Gazebo Launch
```bash
# Terminal 1 - Launch Gazebo with simple test (no MPC yet)
roslaunch gazebo_ros empty_world.launch &
sleep 3
roslaunch uav_simulator start.launch  # Just spawn drone

# Check if drone is controllable
bash /root/ip-mpc_ws/src/Intent-MPC/docker/test_gazebo_flight.sh
```

### 4. Test Full System with DYNUS Obstacles
```bash
# Launch everything
roslaunch autonomous_flight intent_mpc_dynus_gazebo.launch num_obstacles:=10

# In another terminal - monitor topics
rostopic hz /CERLAB/quadcopter/odom  # Should be ~30 Hz
rostopic hz /gazebo/model_states      # Obstacle states
rostopic hz /mpcNavigation/mpc_trajectory  # MPC output

# Check if drone is moving
rostopic echo /CERLAB/quadcopter/pose --noarr

# Check if MPC is avoiding obstacles
# Look for orange line (MPC trajectory) deviating from green line (reference) in RViz
```

## Debugging Checklist

### If Drone Doesn't Take Off:
1. Check Gazebo plugin state
```bash
# Publish takeoff manually
rostopic pub -1 /CERLAB/quadcopter/takeoff std_msgs/Empty

# Then send position target
rostopic pub -1 /CERLAB/quadcopter/setpoint_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 3.0}}}"

# Monitor if z position increases
watch -n 0.5 'rostopic echo /CERLAB/quadcopter/odom/pose/pose/position/z -n 1'
```

2. Check if tracking controller is publishing commands
```bash
rostopic hz /CERLAB/quadcopter/cmd_acc  # Should publish when active
rostopic echo /CERLAB/quadcopter/cmd_acc -n 1  # Check values
```

### If Drone Moves But Doesn't Avoid Obstacles:
1. Check if obstacles are detected
```bash
rostopic hz /fake_detector/dynamic_obstacles  # Should be publishing
rostopic echo /fake_detector/dynamic_obstacles -n 1  # Check obstacle count
```

2. Check if predictor is running
```bash
rostopic hz /dynamic_predictor/predicted_trajectories
```

3. Check MPC trajectory in RViz
- Reference trajectory (green line) should be straight
- MPC trajectory (orange line) should deviate around obstacles
- If both are straight, MPC isn't avoiding

## Key Timing Issues

The launch file uses these delays:
- `sleep 2`: Send takeoff command (transition Gazebo plugin state)
- `sleep 3`: Start mpc_navigation_node (after Gazebo ready)

If takeoff still fails, increase these delays in the launch file:
```xml
<!-- Edit: Intent-MPC/autonomous_flight/launch/intent_mpc_dynus_gazebo.launch -->
<node ... launch-prefix="bash -c 'sleep 5; $0 $@'"/>  <!-- Increase from 2 or 3 -->
```

Then just `catkin_make && source devel/setup.bash` (no Docker rebuild needed!)
