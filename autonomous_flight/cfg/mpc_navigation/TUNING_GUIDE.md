# MPC Obstacle Avoidance Tuning Guide

## Key Parameters (in `planner_param.yaml`)

### 1. Safety Distances (PRIMARY)
```yaml
mpc_planner/static_safety_dist: 0.8   # Distance from static obstacles (m)
mpc_planner/dynamic_safety_dist: 0.6  # Distance from dynamic obstacles (m)
```
**To be less conservative:**
- Decrease these values (e.g., 0.5 for static, 0.4 for dynamic)
- Minimum recommended: 0.3m (size of drone)

### 2. Constraint Slack (SECONDARY)
```yaml
mpc_planner/static_constraint_slack_ratio: 0.01   # Softness of static constraints
mpc_planner/dynamic_constraint_slack_ratio: 0.2   # Softness of dynamic constraints
```
**To be less conservative:**
- Increase slack ratios (allows minor violations)
- Higher = softer constraints = more aggressive
- Try: 0.05 for static, 0.3-0.4 for dynamic

### 3. Speed Limits
```yaml
# In flight_base.yaml
desired_velocity: 1.5      # m/s - max speed
desired_acceleration: 1.5  # m/s^2 - max acceleration
```
**To be more aggressive:**
- Increase velocity (e.g., 2.0-2.5 m/s)
- Increase acceleration (e.g., 2.0-3.0 m/s^2)

### 4. Planning Horizon
```yaml
mpc_planner/horizon: 30  # Number of steps (at 0.1s = 3 seconds lookahead)
```
**Effect:**
- Longer horizon = smoother, more conservative
- Shorter horizon = more reactive, aggressive
- Try: 20-25 for more aggressive behavior

### 5. Local Planning Region
```yaml
mpc_planner/local_cloud_region_x: 10.0  # Forward lookahead (m)
mpc_planner/local_cloud_region_y: 8.0   # Side corridor width (m)
```
**To be more aggressive:**
- Decrease corridor width (e.g., 6.0 for y)
- Allows tighter maneuvers

## Recommended Aggressive Settings

For **moderately aggressive** avoidance:
```yaml
# planner_param.yaml
mpc_planner/static_safety_dist: 0.5
mpc_planner/dynamic_safety_dist: 0.4
mpc_planner/static_constraint_slack_ratio: 0.05
mpc_planner/dynamic_constraint_slack_ratio: 0.3
mpc_planner/horizon: 25
mpc_planner/local_cloud_region_y: 6.0

# flight_base.yaml
desired_velocity: 2.0
desired_acceleration: 2.0
```

For **very aggressive** avoidance (risky!):
```yaml
# planner_param.yaml
mpc_planner/static_safety_dist: 0.4
mpc_planner/dynamic_safety_dist: 0.35
mpc_planner/static_constraint_slack_ratio: 0.1
mpc_planner/dynamic_constraint_slack_ratio: 0.4
mpc_planner/horizon: 20
mpc_planner/local_cloud_region_y: 5.0

# flight_base.yaml
desired_velocity: 2.5
desired_acceleration: 2.5
```

## Testing Workflow

1. Edit parameters in the files (no Docker rebuild needed!)
2. In container: `catkin_make && source devel/setup.bash`
3. Restart: `tmux kill-session -t intent-mpc-dynus && tmuxp load ...`
4. Observe behavior in RViz
5. Iterate until satisfied

## What to Watch

- **Too conservative**: Wide berth around obstacles, slow progress
- **Too aggressive**: Frequent collision warnings, close calls
- **Just right**: Smooth weaving through obstacles, steady progress

Start with moderate settings and gradually make more aggressive!
