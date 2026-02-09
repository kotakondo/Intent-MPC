# DYNUS Obstacles Integration with Intent-MPC

## Overview

This integration ports DYNUS's dynamic obstacle generation system from ROS2 to ROS1, enabling Intent-MPC to run in the exact same environment as DYNUS for fair benchmarking comparisons.

**Key Features:**
- ✅ No Gazebo required
- ✅ Perfect ground truth odometry
- ✅ Same obstacle trajectories as DYNUS
- ✅ Static + dynamic obstacles
- ✅ RViz-only visualization

## Architecture

### Components

1. **dynus_obstacles_node** (`dynus_obstacles_ros1` package)
   - Generates obstacles with trefoil knot trajectories
   - Publishes to `/gazebo/model_states` (gazebo_msgs/ModelStates)
   - Static obstacles (vertical pillars + horizontal walls) with velocity = 0
   - Dynamic obstacles following parametric curves
   - Publishing rate: 50 Hz

2. **fake_odom_node** (`dynus_obstacles_ros1` package)
   - Perfect ground truth odometry
   - Follows reference trajectory exactly
   - Publishes to `/CERLAB/quadcopter/odom` and `/CERLAB/quadcopter/pose`
   - Publishing rate: 100 Hz

3. **Intent-MPC** (existing)
   - Uses fake_detector to read obstacle states
   - MPC planning with dynamic obstacle avoidance
   - Straight-line trajectory: (0,0,3) → (105,0,3)

### Data Flow

```
┌─────────────────────┐
│ dynus_obstacles_node│
│  - Trefoil curves   │
│  - Static obstacles │
└──────────┬──────────┘
           │ /gazebo/model_states
           │ (50 Hz)
           ↓
    ┌──────────────┐
    │ fake_detector│
    └──────┬───────┘
           │
           ↓
    ┌──────────────┐     ┌────────────────┐
    │ Intent-MPC   │←────│ fake_odom_node │
    │  - Predictor │     │ (ground truth) │
    │  - MPC       │     └────────────────┘
    └──────────────┘
```

## Environment Parameters

**Spatial Range** (matching DYNUS):
- X: [5, 105] meters (100m corridor)
- Y: [-15, 15] meters (30m width)
- Z: [0, 7] meters (7m height)

**Drone Trajectory:**
- Start: (0, 0, 3)
- End: (105, 0, 3)
- Velocity: 1.5 m/s
- Duration: ~70 seconds

**Obstacles:**
- Total: Configurable (default: 10)
- Dynamic ratio: Configurable (default: 50%)
- Dynamic: Trefoil knot trajectories, bbox: [0.8, 0.8, 0.8]m
- Static vertical: Pillars, bbox: [0.4, 0.4, 4.0]m (35% of static)
- Static horizontal: Walls, bbox: [0.4, 4.0, 0.4]m (65% of static)

## Usage

### Quick Start

```bash
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker

# Build Docker image (first time only)
make build

# Run simulation with DYNUS obstacles
make run-dynus
```

### With Custom Parameters

```bash
# Inside Docker container
cd /root/ip-mpc_ws
source devel/setup.bash

# Custom number of obstacles
roslaunch autonomous_flight intent_mpc_dynus_sim.launch \
    num_obstacles:=20 \
    dynamic_ratio:=0.7 \
    seed:=42
```

### Using the Run Script

```bash
# Inside Docker container
cd /root/ip-mpc_ws/src/Intent-MPC/scripts

# Default parameters
./run_dynus_sim.sh

# Custom parameters
./run_dynus_sim.sh --num-obstacles 20 --dynamic-ratio 0.7 --seed 42

# Help
./run_dynus_sim.sh --help
```

## Verification

### Check Topics

```bash
# Obstacle states (should be ~50 Hz)
rostopic hz /gazebo/model_states

# Odometry (should be ~100 Hz)
rostopic hz /CERLAB/quadcopter/odom

# MPC trajectory
rostopic echo /mpcNavigation/mpc_trajectory --noarr -n 1

# Obstacle count
rostopic echo /gazebo/model_states -n 1 | grep -c "obstacle_"
```

### RViz Visualization

In RViz, add:
1. **PointCloud2** - `/occupancy_map/voxel_map` (if using map)
2. **MarkerArray** - `/dynus_obstacles/markers` (obstacles)
   - Red cubes: Dynamic obstacles
   - Blue: Static obstacles
3. **Path** - `/mpcNavigation/mpc_trajectory` (planned path)
4. **Odometry** - `/CERLAB/quadcopter/odom` (drone position)

### Expected Behavior

1. **Startup:**
   - 10 obstacles spawned (5 dynamic, 5 static by default)
   - Drone starts at (0, 0, 3)
   - Dynamic obstacles moving in trefoil patterns
   - Static obstacles fixed in place

2. **Navigation:**
   - Drone follows straight-line trajectory
   - MPC avoids detected obstacles
   - Predictor estimates future obstacle positions
   - Perfect odometry (no drift)

3. **Completion:**
   - Drone reaches approximately (105, 0, 3)
   - Total time: ~70 seconds

## Configuration Files

### Obstacle Generation

`/home/kkondo/code/ip-mpc_ws/Intent-MPC/dynus_obstacles_ros1/launch/dynus_obstacles.launch`

```xml
<arg name="num_obstacles" default="10"/>
<arg name="dynamic_ratio" default="0.5"/>
<arg name="seed" default="0"/>
<arg name="publish_rate_hz" default="50.0"/>
<arg name="x_min" default="5.0"/>
<arg name="x_max" default="105.0"/>
<!-- ... -->
```

### Trajectory

`/home/kkondo/code/ip-mpc_ws/Intent-MPC/autonomous_flight/cfg/mpc_navigation/ref_trajectory_straight_line.txt`

- 701 waypoints
- 0.1s timestep
- Linear interpolation

### Intent-MPC Settings

`flight_base.yaml`:
```yaml
takeoff_height: 3.0
predefined_goal_directory: "/cfg/mpc_navigation/ref_trajectory_straight_line.txt"
execute_path_times: 1
desired_velocity: 1.5
```

`mapping_param.yaml`:
```yaml
depth_image_topic: /no_topic  # Disabled - no depth camera
prebuilt_map_directory: "No"  # No static map
map_size: [130, 50, 10]
```

## Comparison with DYNUS

### Similarities (for fair benchmarking)

| Aspect | DYNUS | Intent-MPC (integrated) |
|--------|-------|-------------------------|
| Obstacle trajectories | Trefoil knot | ✅ Same trefoil math |
| Obstacle sizes | 0.8×0.8×0.8m cubes | ✅ Same |
| Environment bounds | [5,105]×[-15,15]×[0,7] | ✅ Same |
| Odometry | Ground truth | ✅ Perfect ground truth |
| Static obstacles | Pillars + walls | ✅ Same (vel=0) |
| Visualization | RViz only | ✅ RViz only |
| Random seed | Controllable | ✅ Controllable |

### Differences

| Aspect | DYNUS | Intent-MPC |
|--------|-------|------------|
| ROS version | ROS2 | ROS1 (ported) |
| Language | Python3 + C++ | C++ only |
| Message format | DynTraj custom | ModelStates (standard) |
| Planning | Global + local | MPC + RRT* |

## Troubleshooting

### Issue: No obstacles appearing

**Check:**
```bash
rostopic echo /gazebo/model_states -n 1
```

**Fix:** Make sure dynus_obstacles_node is running:
```bash
rosnode list | grep dynus
```

### Issue: Drone not moving

**Check:**
```bash
rostopic echo /CERLAB/quadcopter/odom --noarr -n 1
```

**Fix:** Verify fake_odom_node loaded trajectory:
```bash
rosnode info /fake_odom
```

### Issue: MPC not avoiding obstacles

**Check:**
```bash
rostopic echo /goal_reached
rostopic hz /gazebo/model_states
```

**Fix:** Ensure fake_detector is configured:
- `use_fake_detector: true` in flight_base.yaml
- Target obstacles: `["obstacle_0", "obstacle_1", ...]` in fake_detector_param.yaml

### Issue: Build errors

**Error:** `dynus_obstacles_ros1 not found`

**Fix:**
```bash
# Inside Docker
cd /root/ip-mpc_ws
catkin_make
source devel/setup.bash
```

## Future Work

### Benchmarking Integration

Create `run_benchmark.py` that:
1. Launches multiple trials with different seeds
2. Collects metrics:
   - Success rate
   - Collision count
   - Path length
   - Computation time
   - Trajectory smoothness
3. Compares with DYNUS results

### Enhanced Ground Truth

Currently uses perfect odometry. Could add:
- Realistic noise model
- IMU simulation
- State estimation comparison

### Scenario Library

Pre-generate obstacle configurations:
- Easy: 5 obstacles, low density
- Medium: 10 obstacles, mixed
- Hard: 20 obstacles, high density
- Validation seeds for reproducibility

## References

- **DYNUS:** Dynamic-aware spline-based Efficient Trajectory Planning
- **Intent-MPC:** Intent Prediction-Driven Model Predictive Control
- **Trefoil Knot:** Parametric curve used for smooth 3D trajectories

## Contact

For issues or questions about this integration:
- Kota Kondo (kkondo@mit.edu)
