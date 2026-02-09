# Next Steps - Dynamic Obstacle Integration

## ✅ Completed
- [x] Port DYNUS obstacle generation to ROS1
- [x] Implement perfect ground truth odometry
- [x] Integrate with Intent-MPC
- [x] Build Docker image
- [x] Verify basic navigation (agent moving toward goal)
- [x] Rename from "dynus" to "dynamic-sim"

## 🔄 Current Step: Verify Obstacle Avoidance

Now that the agent is moving, we need to verify that the MPC is actually **detecting and avoiding obstacles**, not just blindly following the reference trajectory.

### Quick Visual Check (in RViz)

While simulation is running, look for:

1. **Dynamic Obstacles (Red cubes)**:
   - Should be moving in smooth trefoil knot patterns
   - 5 obstacles total by default
   - Moving continuously, not static

2. **Static Obstacles (Blue cubes)**:
   - Should be stationary
   - 5 obstacles total by default (3 vertical pillars, 2 horizontal walls)
   - Fixed positions

3. **MPC Trajectory (Green/colored path)**:
   - Should **curve around obstacles** when they're nearby
   - If obstacles are far away, may follow straight line
   - Updates in real-time as obstacles move

4. **Drone (coordinate frame)**:
   - Should follow the curved MPC trajectory
   - Should NOT collide with obstacles

### Automated Verification

While simulation is running, **open a new terminal** and run:

```bash
# Get into the running Docker container
docker exec -it $(docker ps -q --filter ancestor=intent-mpc) bash

# Inside the container
cd /root/ip-mpc_ws
source devel/setup.bash
/root/ip-mpc_ws/src/Intent-MPC/scripts/verify_obstacle_avoidance.sh
```

This will check:
- ✓ Obstacles are being detected
- ✓ Predictor is generating future trajectories for dynamic obstacles
- ✓ MPC is generating collision-free paths
- ✓ Drone is deviating from straight line (when avoiding obstacles)

### Manual Topic Checks

```bash
# Check detected obstacles
rostopic echo /fake_detector/dynamic_obstacles

# Check predicted future positions
rostopic echo /dynamic_predictor/predicted_trajectories

# Check MPC planned trajectory
rostopic echo /mpcNavigation/mpc_trajectory

# Check current drone position
rostopic echo /CERLAB/quadcopter/odom/pose/pose/position

# Monitor obstacle positions
rostopic echo /gazebo/model_states
```

## 📊 Next: Data Collection & Benchmarking

Once obstacle avoidance is verified, the next steps are:

### 1. Collect Performance Metrics

Create a benchmarking script that records:
- **Success rate**: Did the drone reach the goal without collision?
- **Path length**: Total distance traveled
- **Path smoothness**: Jerk, acceleration metrics
- **Computation time**: MPC solve time per iteration
- **Closest approach**: Minimum distance to obstacles
- **Collision count**: Number of collisions (if any)

### 2. Run Multiple Trials

Test with different configurations:
```bash
# Easy scenario (5 obstacles)
roslaunch autonomous_flight intent_mpc_dynus_sim.launch \
    num_obstacles:=5 \
    dynamic_ratio:=0.3 \
    seed:=1

# Medium scenario (10 obstacles, default)
roslaunch autonomous_flight intent_mpc_dynus_sim.launch \
    num_obstacles:=10 \
    dynamic_ratio:=0.5 \
    seed:=2

# Hard scenario (20 obstacles, high density)
roslaunch autonomous_flight intent_mpc_dynus_sim.launch \
    num_obstacles:=20 \
    dynamic_ratio:=0.7 \
    seed:=3
```

### 3. Compare with DYNUS

Since DYNUS uses the same obstacle trajectories and environment:
- Run Intent-MPC in multiple scenarios
- Compare metrics with DYNUS paper results
- Analyze strengths/weaknesses of each approach

### 4. Tune Parameters (if needed)

If collision avoidance isn't working well, adjust:
- `static_safety_dist` (default: 0.8m) - in `planner_param.yaml`
- `dynamic_safety_dist` (default: 0.6m) - in `planner_param.yaml`
- MPC horizon (default: 30) - in `planner_param.yaml`
- Prediction size (default: 30) - in `predictor_param.yaml`

## 🚀 Command Reference

```bash
# Build (after code changes)
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker
make build

# Run simulation
make run-dynamic-sim

# Verify obstacle avoidance (while running)
docker exec -it $(docker ps -q --filter ancestor=intent-mpc) bash
source /root/ip-mpc_ws/devel/setup.bash
/root/ip-mpc_ws/src/Intent-MPC/scripts/verify_obstacle_avoidance.sh
```

## 📝 What to Report/Document

When you're ready to benchmark:
1. Screenshot of RViz showing obstacle avoidance
2. Success rate across different seeds (e.g., 10 trials)
3. Average path length vs straight-line distance
4. Computation time statistics (mean, max, std)
5. Any collisions or near-misses

This will allow fair comparison with DYNUS baseline.
