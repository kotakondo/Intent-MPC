# Test Status Report - Dynamic Obstacle Integration

**Date**: 2026-02-04
**Status**: ✅ **CRASH FIXED** - System launches without errors!

---

## 🎉 What's Fixed

### 1. Obstacle Name Format ✅
- **Problem**: `std::invalid_argument` from `stod` - obstacle names had incorrect format
- **Solution**: Changed format to `obstacleXXXYYYZZZ` (no underscore before size encoding)
- **Result**: NO MORE CRASHES!

### 2. Obstacle Detection Configuration ✅
- **Problem**: fake_detector wasn't tracking DYNUS obstacles
- **Solution**: Set `target_obstacle: ["obstacle_"]` for prefix matching
- **Result**: Should detect all obstacles now

### 3. Spatial Range ✅
- **Configured**: X[0,100]m, Y[-7,7]m, Z[0.5,4.5]m (tighter corridor)
- **Result**: Obstacles spawn closer to drone's path

### 4. Visualization ✅
- **Reference Trajectory**: Green line (straight) - `/mpcNavigation/input_trajectory`
- **MPC Planned Trajectory**: Orange line (curves around obstacles) - `/mpcNavigation/mpc_trajectory`
- **Dynamic Obstacles**: Red cubes (moving)
- **Static Obstacles**: Blue cubes (stationary)

---

## 🚀 Ready to Test!

### Command to Run
```bash
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker
make run-dynamic-sim NUM_OBSTACLES=30 DYNAMIC_RATIO=0.7 SEED=42
```

### What to Look For in RViz

1. **Green straight line** - Reference trajectory from (0,0,3) to (100,0,3)
2. **Orange curved line** - MPC planned path avoiding obstacles
3. **Red cubes** - Dynamic obstacles moving in trefoil patterns
4. **Blue cubes** - Static obstacles (pillars and walls)
5. **Drone** - Should follow the orange path, not the green one

### Expected Behavior

✅ **System starts without crashing**
✅ **Nodes running**: dynus_obstacles, fake_odom, mpc_navigation_node, tracking_controller_node
✅ **Topics publishing**:
   - `/gazebo/model_states` (50 Hz) - Obstacle positions
   - `/CERLAB/quadcopter/odom` (100 Hz) - Drone odometry
   - `/mpcNavigation/mpc_trajectory` - Planned path
   - `/fake_detector/dynamic_obstacles` - Detected obstacles

✅ **Orange path curves** around obstacles when they're nearby
✅ **Drone follows** the orange path, avoiding collisions

---

## 🔍 Verification Commands

While simulation is running, open a new terminal:

```bash
# Check nodes are running
docker exec -it $(docker ps -q --filter ancestor=intent-mpc) bash -c \
    "source /root/ip-mpc_ws/devel/setup.bash && rosnode list"

# Check detected obstacles
docker exec -it $(docker ps -q --filter ancestor=intent-mpc) bash -c \
    "source /root/ip-mpc_ws/devel/setup.bash && rostopic echo /fake_detector/dynamic_obstacles -n 1"

# Check MPC trajectory
docker exec -it $(docker ps -q --filter ancestor=intent-mpc) bash -c \
    "source /root/ip-mpc_ws/devel/setup.bash && rostopic echo /mpcNavigation/mpc_trajectory --noarr -n 1 | head -30"

# Check obstacle names (verify format)
docker exec -it $(docker ps -q --filter ancestor=intent-mpc) bash -c \
    "source /root/ip-mpc_ws/devel/setup.bash && rostopic echo /gazebo/model_states/name -n 1"
```

Expected obstacle name format: `["obstacle080080080", "obstacle040040400", ...]`

---

## 📊 Test Results from Automated Run

```
✅ No crashes detected
✅ System initializes successfully
✅ "Takeoff succeed!" message appears
✅ All parameter configurations loaded correctly
```

---

## 🎯 What to Test When You're Back

1. **Launch the simulation** with the command above
2. **Watch RViz** - Does the orange line curve around obstacles?
3. **Check terminal** - Are there any error messages?
4. **Verify avoidance** - Is the drone actually avoiding obstacles?

If the orange MPC trajectory is still a straight line (not curving), we may need to:
- Check if obstacles are being detected: `rostopic echo /fake_detector/dynamic_obstacles`
- Check if predictor is running: `rostopic echo /dynamic_predictor/predicted_trajectories`
- Verify obstacles are in the drone's path (within the corridor)

---

## 🔧 All Changes Made

### Files Modified
1. `/home/kkondo/code/ip-mpc_ws/Intent-MPC/dynus_obstacles_ros1/src/dynus_obstacles_node.cpp`
   - Obstacle name format: `obstacle080080080` (last 9 chars numeric)

2. `/home/kkondo/code/ip-mpc_ws/Intent-MPC/autonomous_flight/cfg/mpc_navigation/fake_detector_param.yaml`
   - `target_obstacle: ["obstacle_"]` (prefix matching)

3. `/home/kkondo/code/ip-mpc_ws/Intent-MPC/dynus_obstacles_ros1/launch/dynus_obstacles.launch`
   - Spatial range: X[0,100], Y[-7,7], Z[0.5,4.5]

4. `/home/kkondo/code/ip-mpc_ws/Intent-MPC/remote_control/rviz/mpc_navigation.rviz`
   - Added MPC planned trajectory display (orange)

5. `/home/kkondo/code/ip-mpc_ws/Intent-MPC/docker/Makefile`
   - Added parameterized `run-dynamic-sim` target

---

## 📝 Quick Reference

```bash
# Run with different densities
make run-dynamic-sim NUM_OBSTACLES=10   # Sparse
make run-dynamic-sim NUM_OBSTACLES=30   # Medium (default)
make run-dynamic-sim NUM_OBSTACLES=50   # Dense

# Run with different dynamic ratios
make run-dynamic-sim DYNAMIC_RATIO=0.3  # 30% dynamic
make run-dynamic-sim DYNAMIC_RATIO=0.7  # 70% dynamic (default)
make run-dynamic-sim DYNAMIC_RATIO=0.9  # 90% dynamic

# Run with different seeds (for reproducibility)
make run-dynamic-sim SEED=1
make run-dynamic-sim SEED=42
make run-dynamic-sim SEED=123
```

---

**Bottom Line**: The system is ready to test! The crashes are fixed, obstacle detection is configured, and visualization is set up. Just run it and verify the MPC is avoiding obstacles.

🎉 Good luck testing! 🎉
