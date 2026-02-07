# Intent-MPC Benchmarking System

Comprehensive benchmarking tools for evaluating Intent-MPC performance with dynamic obstacles. Compatible with DYNUS benchmark format for direct comparison.

## Overview

The benchmarking system consists of two main scripts:

1. **`run_mpc_benchmark.py`** - Runs multiple trials and collects performance data
2. **`analyze_mpc_benchmark.py`** - Analyzes collected data and generates reports

## Quick Start

### Running Benchmarks

```bash
# From host machine (recommended)
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker
make run-benchmark NUM_TRIALS=20 NUM_OBSTACLES=200 DYNAMIC_RATIO=0.7 SEED_START=0

# Or manually inside Docker container
docker exec -it <container> bash
cd /root/ip-mpc_ws
source devel/setup.bash
python3 src/Intent-MPC/scripts/run_mpc_benchmark.py \
    --num-trials 20 \
    --num-obstacles 200 \
    --dynamic-ratio 0.7 \
    --seed-start 0
```

### Analyzing Results

```bash
# Analyze benchmark data
python3 scripts/analyze_mpc_benchmark.py --data-dir data/benchmark_20260205_120000

# Custom output names
python3 scripts/analyze_mpc_benchmark.py \
    --data-dir data/benchmark_20260205_120000 \
    --output-name my_results \
    --latex-name my_table.tex \
    --algorithm-name "Intent-MPC-v2"
```

## Collected Metrics

### Success Metrics
- **Goal Reached**: Whether the drone reached the goal position (105, 0, 3) within threshold
- **Timeout**: Whether the trial exceeded the time limit (default 120s)
- **Collision**: Whether any collision was detected during the flight

### Timing Metrics
- **Flight Travel Time**: Total time from start to goal (or timeout)
- **Replanning Time**: Average and maximum replanning computation time
- **Number of Replans**: Total replanning operations during flight

### Path Metrics
- **Path Length**: Total distance traveled along the trajectory
- **Straight Line Distance**: Euclidean distance from start to goal
- **Path Efficiency**: Ratio of straight-line distance to actual path length (1.0 = optimal)

### Smoothness Metrics
- **Jerk RMS**: Root mean square jerk (derivative of acceleration)
- **Jerk Integral**: Integrated jerk over the entire trajectory
- **Average/Max Velocity**: Velocity statistics
- **Average/Max Acceleration**: Acceleration statistics

### Constraint Violations
- **Velocity Violations**: Count of samples exceeding velocity limit (2.0 m/s)
- **Acceleration Violations**: Count of samples exceeding acceleration limit (2.0 m/s²)
- **Jerk Violations**: Count of samples exceeding jerk limit (20.0 m/s³)

### Safety Metrics
- **Collision Count**: Total number of collision events
- **Collision Penetration**: Maximum penetration depth during collisions
- **Min Distance to Obstacles**: Minimum distance maintained to any obstacle
- **Collision-Free Ratio**: Fraction of trajectory without collisions

## Data Format

### CSV Output

Each trial is recorded as a row with the following columns:

```
trial_id, seed, num_obstacles, dynamic_ratio, timestamp,
goal_reached, timeout_reached, collision,
flight_travel_time, total_replanning_time,
num_replans, avg_replanning_time, max_replanning_time,
path_length, straight_line_distance, path_efficiency,
jerk_integral, jerk_rms, avg_velocity, max_velocity,
avg_acceleration, max_acceleration,
vel_violation_count, acc_violation_count, jerk_violation_count,
collision_count, collision_penetration_max, min_distance_to_obstacles,
collision_free_ratio, start_position, goal_position
```

### Directory Structure

```
Intent-MPC/
├── data/
│   └── benchmark_YYYYMMDD_HHMMSS/
│       ├── benchmark_intent_mpc_YYYYMMDD_HHMMSS.csv    # Raw trial data
│       ├── benchmark_intent_mpc_YYYYMMDD_HHMMSS.json   # JSON format
│       ├── mpc_benchmark_summary.csv                    # Statistical summary
│       └── intent_mpc_benchmark.tex                     # LaTeX table
└── scripts/
    ├── run_mpc_benchmark.py
    ├── analyze_mpc_benchmark.py
    └── BENCHMARK_README.md
```

## Configuration Options

### Benchmark Runner Options

```bash
python3 scripts/run_mpc_benchmark.py [OPTIONS]

Required:
  --num-trials N         Number of trials to run (default: 20)
  --num-obstacles N      Number of obstacles in environment (default: 200)
  --dynamic-ratio R      Ratio of dynamic obstacles (0.0-1.0, default: 0.7)

Optional:
  --seed-start N         Starting seed value (default: 0)
  --timeout T            Timeout per trial in seconds (default: 120)
  --output-dir DIR       Output directory (default: /root/ip-mpc_ws/src/Intent-MPC/data)
  --headless             Run without Gazebo GUI (default: True)
  --gui                  Run with Gazebo GUI (for debugging)
```

### Analysis Options

```bash
python3 scripts/analyze_mpc_benchmark.py [OPTIONS]

Required:
  --data-dir DIR         Path to benchmark data directory

Optional:
  --output-name NAME     Output CSV filename prefix (default: mpc_benchmark_summary)
  --latex-name NAME      LaTeX table filename (default: intent_mpc_benchmark.tex)
  --algorithm-name NAME  Algorithm name for table (default: Intent-MPC)
```

## Comparison with DYNUS

The benchmark format is designed to be directly comparable with DYNUS benchmarks:

| Metric | Intent-MPC | DYNUS | Notes |
|--------|------------|-------|-------|
| Success Rate | ✓ | ✓ | Same definition |
| Collision-Free Rate | ✓ | ✓ | Same definition |
| Path Length | ✓ | ✓ | Same calculation |
| Jerk Integral | ✓ | ✓ | Same smoothness metric |
| Min Distance | ✓ | ✓ | Safety metric |
| Constraint Violations | ✓ | ✓ | Velocity, acceleration, jerk |
| Replanning Time | ✓ | ✓ | Computation efficiency |

## LaTeX Table Output

The analysis script generates a LaTeX table compatible with the DYNUS paper format:

```latex
\begin{table*}
  \caption{Intent-MPC benchmarking results...}
  \begin{tabular}{c c c c c c c c c c c}
    Algorithm & R^succ [%] & R^coll_free [%] & T^per_opt [ms] & ...
    Intent-MPC & 95.0 & 87.5 & 12.3 & ...
  \end{tabular}
\end{table*}
```

## Example Usage

### Run 50 trials with 200 obstacles

```bash
cd Intent-MPC/docker
make run-benchmark NUM_TRIALS=50 NUM_OBSTACLES=200 DYNAMIC_RATIO=0.7 SEED_START=0
```

This will:
1. Run 50 trials with seeds 0-49
2. Each trial has 200 obstacles (70% dynamic, 30% static)
3. Save results to `data/benchmark_<timestamp>/`
4. Data persists on host machine even after container stops

### Analyze and compare results

```bash
# Inside the Intent-MPC directory
python3 scripts/analyze_mpc_benchmark.py --data-dir data/benchmark_20260205_120000

# This generates:
# - Console summary with statistics
# - mpc_benchmark_summary.csv (for spreadsheets)
# - intent_mpc_benchmark.tex (for papers)
```

### Compare with DYNUS

```bash
# Run DYNUS benchmark
cd /home/kkondo/code/dynus_ws/src/dynus
python3 scripts/run_benchmark.py --num-trials 50 ...
python3 scripts/analyze_dynamic_benchmark.py --data-dir benchmark_data/default/...

# Run Intent-MPC benchmark
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC
make run-benchmark NUM_TRIALS=50 ...
python3 scripts/analyze_mpc_benchmark.py --data-dir data/benchmark_...

# Compare the generated LaTeX tables or CSV summaries
```

## Troubleshooting

### No trajectory data collected

**Symptom**: All trials show `path_length=0.0`

**Causes**:
- ROS topics not publishing (check with `rostopic list`)
- Navigation node crashed during trial
- Simulation failed to initialize

**Fix**:
```bash
# Check topics inside container
rostopic list | grep -E "odom|trajectory"
rostopic hz /CERLAB/quadcopter/odom

# Run a single trial with GUI to debug
python3 scripts/run_mpc_benchmark.py --num-trials 1 --gui
```

### Benchmark crashes after first trial

**Symptom**: Process cleanup fails between trials

**Fix**:
```bash
# Increase delay between trials in run_mpc_benchmark.py
# Edit line ~700:
time.sleep(5)  # Increase from 3 to 5 seconds

# Or manually kill processes
pkill -9 gazebo
pkill -9 roslaunch
```

### ROS node initialization errors

**Symptom**: `rospy.init_node()` called multiple times

**Fix**:
- The benchmark script initializes a new node for each trial
- Make sure no other ROS nodes are running with the same name
- Use `rosnode cleanup` between runs

### Data not persisting on host

**Symptom**: Data disappears after container stops

**Fix**:
```bash
# Make sure data folder is mounted correctly
# Check Makefile line ~140:
--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw

# Verify mount inside container:
docker exec -it <container> ls -la /root/ip-mpc_ws/src/Intent-MPC/data
```

## Advanced Usage

### Custom Obstacle Configurations

Test with different obstacle scenarios:

```bash
# Sparse environment (100 obstacles, 50% dynamic)
make run-benchmark NUM_OBSTACLES=100 DYNAMIC_RATIO=0.5 NUM_TRIALS=30

# Dense environment (300 obstacles, 80% dynamic)
make run-benchmark NUM_OBSTACLES=300 DYNAMIC_RATIO=0.8 NUM_TRIALS=30

# Static-only (200 obstacles, 0% dynamic)
make run-benchmark NUM_OBSTACLES=200 DYNAMIC_RATIO=0.0 NUM_TRIALS=30
```

### Reproducible Experiments

Use specific seed ranges for reproducibility:

```bash
# Experiment 1: Seeds 0-49
make run-benchmark NUM_TRIALS=50 SEED_START=0

# Experiment 2: Seeds 50-99 (different random configurations)
make run-benchmark NUM_TRIALS=50 SEED_START=50

# Experiment 3: Repeat seeds 0-49 (should get same results)
make run-benchmark NUM_TRIALS=50 SEED_START=0
```

### Parallel Benchmarking

Run multiple benchmark sessions with different configurations:

```bash
# Terminal 1: Dense obstacles
make run-benchmark NUM_OBSTACLES=300 SEED_START=0 NUM_TRIALS=20

# Terminal 2: Sparse obstacles (after Terminal 1 completes)
make run-benchmark NUM_OBSTACLES=100 SEED_START=0 NUM_TRIALS=20

# Analyze both
python3 scripts/analyze_mpc_benchmark.py --data-dir data/benchmark_<timestamp1> --output-name dense_results
python3 scripts/analyze_mpc_benchmark.py --data-dir data/benchmark_<timestamp2> --output-name sparse_results
```

## References

- DYNUS benchmark: `/home/kkondo/code/dynus_ws/src/dynus/scripts/`
- Intent-MPC paper: TBD
- ROS1 documentation: http://wiki.ros.org/rospy

## Support

For issues or questions:
1. Check logs in Docker container: `docker logs <container>`
2. Verify ROS topics: `rostopic list`, `rostopic hz`
3. Test single trial with GUI: `--gui` flag
4. Check GitHub issues: https://github.com/kotakondo/Intent-MPC/issues
