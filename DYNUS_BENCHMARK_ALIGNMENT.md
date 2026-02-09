# DYNUS Benchmark Alignment Summary

This document summarizes all configuration changes made to Intent-MPC to ensure **fair comparison** with DYNUS benchmarks.

## Overview

All parameters have been updated to **exactly match** the DYNUS benchmark configuration for fair, apples-to-apples comparison.

---

## 1. Motion Constraints

### DYNUS Configuration (from dynus.yaml)
```yaml
v_max: 5.0   # m/s
a_max: 10.0  # m/s²
j_max: 50.0  # m/s³
```

### Intent-MPC Updates

**File**: `autonomous_flight/cfg/mpc_navigation/flight_base.yaml`

```yaml
# BEFORE (TOO RESTRICTIVE - NOT FAIR):
desired_velocity: 2.0       # m/s
desired_acceleration: 2.0   # m/s²
# No jerk constraint

# AFTER (MATCHED TO DYNUS):
desired_velocity: 5.0       # m/s - DYNUS benchmark constraint
desired_acceleration: 10.0  # m/s² - DYNUS benchmark constraint
desired_angular_velocity: 1.0  # rad/s - DYNUS benchmark constraint
```

**Note**: Intent-MPC does not have an explicit jerk constraint in the MPC formulation, but the benchmark script tracks jerk violations against the 50.0 m/s³ limit for comparison purposes.

---

## 2. Start and Goal Positions

### DYNUS Configuration
```python
start: [0.0, 0.0, 2.0]  # meters
goal:  [105.0, 0.0, 2.0]  # meters
```

### Intent-MPC Updates

**Files Updated**:
1. `autonomous_flight/cfg/mpc_navigation/flight_base.yaml`
   - `takeoff_height: 2.0` (was 3.0)
   - `predefined_goal_directory: "/cfg/mpc_navigation/ref_trajectory_dynus_benchmark.txt"`

2. Created new trajectory file: `ref_trajectory_dynus_benchmark.txt`
   - 43 waypoints from (0,0,2) to (105,0,2)
   - Spacing: 2.5m between waypoints
   - Total distance: 105.0m

3. `scripts/run_mpc_benchmark.py`
   - `start_position: [0.0, 0.0, 2.0]`
   - `goal_position: [105.0, 0.0, 2.0]`

---

## 3. Environment Configuration

### Obstacle Spawn Range

**DYNUS Configuration**:
```python
x_min: 5.0, x_max: 105.0   # 100m corridor length
y_min: -15.0, y_max: 15.0  # 30m corridor width
z_min: 0.0, z_max: 6.0     # 6m corridor height
```

**Intent-MPC Updates** (`dynus_obstacles_ros1/launch/dynus_obstacles.launch`):

```xml
<!-- BEFORE (WRONG RANGE):
x_min: 0.0, x_max: 100.0
y_min: -7.0, y_max: 7.0
z_min: 0.5, z_max: 4.5
-->

<!-- AFTER (MATCHED TO DYNUS): -->
<arg name="x_min" default="5.0"/>
<arg name="x_max" default="105.0"/>
<arg name="y_min" default="-15.0"/>
<arg name="y_max" default="15.0"/>
<arg name="z_min" default="0.0"/>
<arg name="z_max" default="6.0"/>
```

### Obstacle Sizes

Both systems use **identical obstacle dimensions**:

```python
Dynamic obstacles:     0.8 × 0.8 × 0.8 m (cubes)
Static vertical:       0.4 × 0.4 × 4.0 m (pillars, 35%)
Static horizontal:     0.4 × 4.0 × 0.4 m (walls, 65%)
```

✓ **Verified**: Obstacle sizes in `dynus_obstacles_node.cpp` match DYNUS exactly.

### Dynamic Obstacle Motion

**DYNUS Configuration**:
```python
slower_min: 10.0  # Trefoil trajectory speed parameter
slower_max: 12.0
```

**Intent-MPC Updates** (`dynus_obstacles.launch`):

```xml
<!-- BEFORE: slower_min: 4.0, slower_max: 6.0 (TOO FAST) -->
<!-- AFTER:  -->
<arg name="slower_min" default="10.0"/>
<arg name="slower_max" default="12.0"/>
```

**Note**: Higher `slower` values = slower obstacle motion (counter-intuitive naming).

---

## 4. Obstacle Distribution

### DYNUS Configuration
```python
num_obstacles: 200 (default)
dynamic_ratio: 0.65 (default)
# Results in: 130 dynamic + 70 static obstacles
```

### Intent-MPC Updates

**Files Updated**:
1. `dynus_obstacles_ros1/launch/dynus_obstacles.launch`
   ```xml
   <arg name="num_obstacles" default="200"/>
   <arg name="dynamic_ratio" default="0.65"/>
   ```

2. `autonomous_flight/launch/intent_mpc_dynus_sim_headless.launch`
   ```xml
   <arg name="num_obstacles" default="200"/>
   <arg name="dynamic_ratio" default="0.65"/>
   ```

3. `scripts/run_mpc_benchmark.py`
   ```python
   --dynamic-ratio: default=0.65
   ```

4. `docker/Makefile`
   ```makefile
   DYNAMIC_RATIO = $(if $(DYNAMIC_RATIO),$(DYNAMIC_RATIO),0.65 (default, DYNUS match))
   ```

---

## 5. Drone Bounding Box

### DYNUS Configuration
```yaml
drone_bbox: [0.2, 0.2, 0.2]  # Full bounding box dimensions
```

### Intent-MPC Updates

**File**: `scripts/run_mpc_benchmark.py`

```python
# BEFORE:
self.drone_half_extents = (0.3, 0.3, 0.15)  # Conservative estimate

# AFTER (MATCHED TO DYNUS):
self.drone_half_extents = (0.1, 0.1, 0.1)  # Half of [0.2, 0.2, 0.2]
```

---

## 6. Benchmark Script Constraint Limits

### Updates in `run_mpc_benchmark.py`

**BenchmarkMetrics dataclass**:

```python
# BEFORE:
vel_limit: float = 2.0   # m/s (too restrictive)
acc_limit: float = 2.0   # m/s² (too restrictive)
jerk_limit: float = 20.0 # m/s³ (too restrictive)

# AFTER (MATCHED TO DYNUS):
vel_limit: float = 5.0   # m/s (DYNUS: v_max)
acc_limit: float = 10.0  # m/s² (DYNUS: a_max)
jerk_limit: float = 50.0 # m/s³ (DYNUS: j_max)
```

---

## Complete Configuration Comparison

| Parameter | DYNUS | Intent-MPC (Before) | Intent-MPC (After) | Status |
|-----------|-------|---------------------|-------------------|--------|
| **Motion Constraints** |
| v_max | 5.0 m/s | 2.0 m/s ❌ | 5.0 m/s ✓ | FIXED |
| a_max | 10.0 m/s² | 2.0 m/s² ❌ | 10.0 m/s² ✓ | FIXED |
| j_max | 50.0 m/s³ | N/A ❌ | 50.0 m/s³ (tracking) ✓ | FIXED |
| **Environment** |
| x_range | [5.0, 105.0] | [0.0, 100.0] ❌ | [5.0, 105.0] ✓ | FIXED |
| y_range | [-15.0, 15.0] | [-7.0, 7.0] ❌ | [-15.0, 15.0] ✓ | FIXED |
| z_range | [0.0, 6.0] | [0.5, 4.5] ❌ | [0.0, 6.0] ✓ | FIXED |
| num_obstacles | 200 | 200 ✓ | 200 ✓ | OK |
| dynamic_ratio | 0.65 | 0.7 ❌ | 0.65 ✓ | FIXED |
| slower_min/max | 10.0-12.0 | 4.0-6.0 ❌ | 10.0-12.0 ✓ | FIXED |
| **Obstacle Sizes** |
| Dynamic | 0.8³ | 0.8³ ✓ | 0.8³ ✓ | OK |
| Static vert | 0.4×0.4×4.0 | 0.4×0.4×4.0 ✓ | 0.4×0.4×4.0 ✓ | OK |
| Static horiz | 0.4×4.0×0.4 | 0.4×4.0×0.4 ✓ | 0.4×4.0×0.4 ✓ | OK |
| **Start/Goal** |
| Start pos | [0,0,2] | [0,0,3] ❌ | [0,0,2] ✓ | FIXED |
| Goal pos | [105,0,2] | [105,0,3] ❌ | [105,0,2] ✓ | FIXED |
| **Drone** |
| Bounding box | [0.2,0.2,0.2] | [0.6,0.6,0.3] ❌ | [0.2,0.2,0.2] ✓ | FIXED |

---

## Files Modified

1. ✓ `autonomous_flight/cfg/mpc_navigation/flight_base.yaml`
2. ✓ `autonomous_flight/cfg/mpc_navigation/ref_trajectory_dynus_benchmark.txt` (NEW)
3. ✓ `dynus_obstacles_ros1/launch/dynus_obstacles.launch`
4. ✓ `autonomous_flight/launch/intent_mpc_dynus_sim_headless.launch`
5. ✓ `scripts/run_mpc_benchmark.py`
6. ✓ `docker/Makefile`

---

## Verification Checklist

- [x] Motion constraints match (v_max, a_max, j_max)
- [x] Start position matches [0, 0, 2]
- [x] Goal position matches [105, 0, 2]
- [x] Obstacle spawn range matches
- [x] Number of obstacles matches (200 default)
- [x] Dynamic ratio matches (0.65 default)
- [x] Obstacle sizes match
- [x] Obstacle motion speed matches (slower_min/max)
- [x] Drone bounding box matches [0.2, 0.2, 0.2]
- [x] Benchmark script constraint limits match

---

## Running Fair Benchmarks

### Option 1: Using Makefile (Recommended)

```bash
cd Intent-MPC/docker

# Run with DYNUS-matched defaults (200 obstacles, 0.65 dynamic ratio)
make run-benchmark NUM_TRIALS=50 SEED_START=0

# This will use:
#   - v_max: 5.0 m/s
#   - a_max: 10.0 m/s²
#   - j_max: 50.0 m/s³ (tracking)
#   - Start: [0, 0, 2]
#   - Goal: [105, 0, 2]
#   - Obstacles: 200 (130 dynamic, 70 static)
#   - Spawn range: x=[5,105], y=[-15,15], z=[0,6]
```

### Option 2: Manual Launch

```bash
# Inside Docker container
cd /root/ip-mpc_ws
source devel/setup.bash

python3 src/Intent-MPC/scripts/run_mpc_benchmark.py \
    --num-trials 50 \
    --num-obstacles 200 \
    --dynamic-ratio 0.65 \
    --seed-start 0
```

### Comparing with DYNUS

Run equivalent DYNUS benchmark:

```bash
cd /home/kkondo/code/dynus_ws/src/dynus

python3 scripts/run_benchmark.py \
    --num-trials 50 \
    --num-obstacles 200 \
    --dynamic-ratio 0.65 \
    --start-seed 0 \
    --setup-bash ../../install/setup.bash
```

Both should now run with **identical**:
- Motion constraints
- Environment setup
- Obstacle distribution
- Start/goal positions
- Drone size

---

## Important Notes

### 1. Jerk Constraint

**DYNUS**: Has explicit jerk constraint (j_max = 50.0 m/s³) in optimization

**Intent-MPC**: Does not have explicit jerk constraint in MPC formulation
- Jerk is **tracked** in benchmark for comparison
- Violations are counted against 50.0 m/s³ limit
- This may give Intent-MPC a slight advantage (smoother trajectories without hard constraint)

### 2. Optimization Approach

**DYNUS**:
- Uses Gurobi MIQP with Safe Flight Corridor (SFC)
- Explicit polytope constraints
- Computation time includes global planning + corridor generation + local trajectory optimization

**Intent-MPC**:
- Uses ACADO MPC with RRT + polynomial trajectory + MPC
- Implicit obstacle avoidance through local point cloud
- Computation time includes RRT planning + polynomial fitting + MPC solve

These are **different algorithms** being compared, but the **environment and constraints are now identical**.

### 3. Collision Detection

Both systems use:
- Axis-aligned bounding boxes (AABB)
- Interpolated obstacle positions
- Same drone bounding box [0.2, 0.2, 0.2]

---

## Summary

✓ All parameters have been updated to match DYNUS benchmark configuration

✓ Environment setup is identical (spawn ranges, obstacle counts, sizes)

✓ Motion constraints are identical (v_max, a_max, j_max tracking)

✓ Start/goal positions match exactly

✓ Fair, apples-to-apples comparison is now possible

The benchmarking system is ready for **direct comparison** with DYNUS results.
