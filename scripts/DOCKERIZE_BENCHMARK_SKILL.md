# Dockerize & Benchmark ROS1 Planner Skill

This document captures the complete workflow for taking a ROS1 Noetic-based motion planner and:
1. Dockerizing it
2. Integrating with DYNUS dynamic obstacle environments
3. Setting up automated benchmarking
4. Generating publication-ready LaTeX tables

---

## How to Use This Skill

### Option 1: Copy the Prompt Template Below

Copy the prompt template from the "Prompt Template" section below and paste it into a new Claude Code session with your target repository.

### Option 2: Create a Claude Code Skill

Add this to your `~/.claude/skills/` directory or project's `.claude/skills/` directory:

```yaml
# ~/.claude/skills/dockerize-benchmark.yaml
name: dockerize-benchmark
description: Dockerize a ROS1 planner and set up DYNUS benchmark infrastructure
trigger: /dockerize-benchmark
prompt: |
  # Task: Dockerize and Benchmark ROS1 Planner

  You are setting up a complete benchmarking infrastructure for a ROS1 Noetic-based motion planner. Follow these phases in order:

  ## Phase 1: Analyze the Repository
  - Identify the main planning package(s)
  - Find the robot model (URDF)
  - Identify sensor topics (depth camera, lidar, odometry)
  - Find the main launch file(s)
  - Identify configuration files (YAML params)
  - Check for existing Docker setup

  ## Phase 2: Create Docker Environment
  Create a docker/ directory with:
  - Dockerfile based on ros:noetic with GUI support
  - Makefile with build, run, shell targets
  - docker-compose.yml or shell scripts for X11 forwarding
  - README.md with usage instructions

  Base Dockerfile template:
  ```dockerfile
  FROM ros:noetic

  ENV DEBIAN_FRONTEND=noninteractive

  # Install dependencies
  RUN apt-get update && apt-get install -y \
      ros-noetic-desktop-full \
      ros-noetic-gazebo-ros-pkgs \
      python3-catkin-tools \
      python3-pip \
      tmux tmuxp \
      git wget curl \
      && rm -rf /var/lib/apt/lists/*

  # Create workspace
  WORKDIR /root/catkin_ws

  # Copy source
  COPY . src/[PACKAGE_NAME]

  # Install ROS dependencies
  RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

  # Build
  RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

  # Setup entrypoint
  COPY docker/entrypoint.sh /entrypoint.sh
  RUN chmod +x /entrypoint.sh
  ENTRYPOINT ["/entrypoint.sh"]
  CMD ["bash"]
  ```

  ## Phase 3: Integrate DYNUS Obstacles
  Clone and integrate the DYNUS obstacle generator:
  ```bash
  git clone https://github.com/kotakondo/dynus_obstacles_ros1.git
  ```

  Create launch file that:
  - Spawns the robot in Gazebo
  - Launches DYNUS obstacle node with configurable parameters:
    - num_obstacles (default: 200)
    - dynamic_ratio (default: 0.65)
    - seed (for reproducibility)
    - spatial bounds matching your environment

  ## Phase 4: Create Benchmark Scripts

  Create scripts/ directory with:

  ### run_benchmark.py
  ```python
  # Key features:
  # - Command-line args: --num-trials, --num-obstacles, --dynamic-ratio, --seed-start
  # - Launches simulation headless (gui:=false)
  # - Records rosbag with key topics
  # - Monitors: goal reached, collisions, timeout
  # - Computes metrics: path length, velocity, acceleration, jerk
  # - Saves results to CSV/JSON
  # - Proper cleanup between trials
  ```

  ### analyze_benchmark.py
  ```python
  # Key features:
  # - Reads benchmark CSV
  # - Computes statistics (mean, std) for successful runs
  # - Success = goal_reached AND collision_free
  # - Generates LaTeX table matching DYNUS format
  # - Extracts MPC/planning computation time from rosbags
  ```

  ## Phase 5: Configure for Your Planner

  Identify and configure:
  - Start/goal positions
  - Velocity/acceleration limits
  - Sensor topics to record
  - Success criteria (goal threshold distance)
  - Timeout duration

  ## Phase 6: Create Benchmark Trajectory

  Generate reference trajectory file with waypoints from start to goal.

  ## Phase 7: Test and Validate

  1. Build Docker: `make build`
  2. Run single trial manually to verify
  3. Run debug mode to monitor behavior
  4. Run full benchmark suite
  5. Analyze results and generate tables

  ---

  Ask me for the repository path and I'll begin the analysis.
```

---

## Prompt Template

Copy and paste this into a new Claude Code session:

```
I have a ROS1 Noetic-based motion planner at: [YOUR_REPO_PATH]

Please help me:
1. Dockerize this ROS1 workspace with GUI support (Gazebo, RViz)
2. Integrate DYNUS dynamic obstacle environments for benchmarking
3. Create automated benchmark scripts that:
   - Run multiple trials with different random seeds
   - Record rosbags for post-processing
   - Track metrics: path length, velocity, acceleration, jerk, collisions, computation time
   - Define success as goal_reached AND collision_free
4. Generate publication-ready LaTeX tables matching DYNUS benchmark format

## DYNUS Benchmark Configuration (standard settings):
- 200 obstacles, 65% dynamic ratio
- Environment: 100m x 30m corridor
- Start: (0, 0, 2), Goal: (105, 0, 2)
- Velocity limit: 5.0 m/s
- Acceleration limit: 10.0 m/s²
- 20 trials per configuration

## Expected Deliverables:
1. docker/ folder with Dockerfile, Makefile, README
2. Integration with dynus_obstacles_ros1 package
3. scripts/run_benchmark.py - automated trial runner
4. scripts/analyze_benchmark.py - results analysis and LaTeX generation
5. Launch files for headless benchmark runs
6. .gitignore excluding data/ folder

Please start by analyzing my repository structure to understand:
- Main planning package and launch files
- Robot model (URDF) and sensor configuration
- Existing dependencies and build system
- Current simulation setup (if any)

Then proceed with the implementation phases.
```

---

## Key Files Created in This Workflow

```
your_planner_ws/
├── docker/
│   ├── Dockerfile
│   ├── Makefile
│   ├── README.md
│   ├── entrypoint.sh
│   └── benchmark_sim.yml          # tmuxp config
├── scripts/
│   ├── run_benchmark.py           # Main benchmark runner
│   ├── analyze_benchmark.py       # Results analysis + LaTeX
│   ├── BENCHMARK_README.md
│   └── ...
├── dynus_obstacles_ros1/          # Submodule or copied
│   ├── src/dynus_obstacles_node.cpp
│   └── launch/dynus_obstacles.launch
├── your_planner_pkg/
│   ├── launch/
│   │   ├── start_dynus.launch     # Gazebo + obstacles
│   │   └── your_nav_dynus.launch  # Navigation for benchmark
│   └── cfg/
│       └── ref_trajectory_dynus_benchmark.txt
├── .gitignore
└── data/                          # Excluded from git
    └── benchmark_YYYYMMDD_HHMMSS/
        ├── benchmark_results.csv
        ├── benchmark_results.json
        └── bags/
```

---

## LaTeX Table Format (DYNUS Standard)

The benchmark produces tables like:

```latex
\begin{table}[h]
\centering
\caption{Benchmark Results (N=200 obstacles, 65\% dynamic)}
\begin{tabular}{l|ccccccccc}
\hline
Method & $R^{succ}$ & $T^{flight}$ & $L^{path}$ & $\bar{v}$ & $\bar{a}$ & $J_{rms}$ & $d_{min}$ & $T^{comp}$ \\
\hline
Your-Method & 85.0\% & 52.3±5.2 & 108.5±3.1 & 2.1±0.3 & 1.5±0.4 & 12.3±2.1 & 0.45±0.12 & 15.2±3.1 \\
\hline
\end{tabular}
\end{table}
```

---

## Common Adaptations Needed

### Different Robot Models
- Update URDF path in launch files
- Adjust sensor topic names
- Modify drone bounding box size for collision detection

### Different Planners
- Identify the "replan" or "compute trajectory" function
- Add timing instrumentation around planning calls
- Publish computation time to a ROS topic

### Different Environments
- Adjust obstacle spatial bounds (x_min, x_max, y_min, y_max, z_min, z_max)
- Modify start/goal positions
- Update map size in configuration

### Different Success Criteria
- Adjust goal_threshold distance
- Modify timeout duration
- Add planner-specific failure conditions

---

## Troubleshooting Common Issues

### Memory Leaks (std::bad_alloc)
- Check for `new` without matching `delete[]`
- Look for accumulating data structures (vectors that grow unbounded)
- Add memory cleanup between trials

### Drone Teleporting
- Check for conflicting odometry publishers
- Ensure clean process shutdown between trials
- Verify spawn position after Gazebo launch

### Collisions Not Detected
- Verify obstacle topic is correct (/dynus/model_states, not /gazebo/model_states)
- Check bounding box sizes
- Ensure collision checking runs at sufficient frequency

### Inconsistent Results
- Use fixed seeds for reproducibility
- Ensure proper cleanup between trials
- Wait for all nodes to fully initialize before starting
