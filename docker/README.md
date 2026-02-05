# Intent-MPC with DYNUS Obstacles

Model Predictive Control (MPC) based navigation for quadrotors with dynamic obstacle avoidance using DYNUS obstacle generation.

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Building](#building)
- [Running Simulations](#running-simulations)
- [Development Workflow](#development-workflow)
- [Git Workflow](#git-workflow)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)

## Overview

This system provides:
- **MPC-based trajectory planning** with obstacle avoidance
- **DYNUS obstacle generation** - configurable dynamic and static obstacles
- **Fake detector** - ground truth obstacle detection for simulation
- **Intent-aware prediction** - anticipates obstacle trajectories
- **Dockerized environment** - consistent ROS Noetic setup

### Architecture
```
DYNUS → /dynus/model_states → fake_detector → MPC Planner → Tracking Controller → Quadrotor
                                                    ↓
                                            Predictor (intent-aware)
```

## Prerequisites

- **Docker**: Install from [docker.com](https://docs.docker.com/get-docker/)
- **nvidia-docker2** (for GPU support): Required for Gazebo visualization
- **Git**: For version control
- **X11 Server**: For GUI display
  - Linux: Built-in
  - macOS: Install [XQuartz](https://www.xquartz.org/)
  - Windows: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)

### System Requirements
- **GPU**: NVIDIA GPU with compatible drivers (for Gazebo/RViz)
- **RAM**: 4GB minimum, 8GB recommended
- **Disk**: 10GB free space
- **CPU**: Multi-core recommended for simulation

## Quick Start

```bash
# 1. Allow Docker to access X server (one-time setup)
xhost +local:docker

# 2. Navigate to docker directory
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker

# 3. Build Docker image (first time only, ~5-10 minutes)
make build

# 4. Run simulation with 200 obstacles (RECOMMENDED)
make run-dynamic-gazebo NUM_OBSTACLES=200 DYNAMIC_RATIO=0.7 SEED=42

# 5. Stop simulation (from another terminal)
make stop
```

## Building

### Initial Build

Build the Docker image with all dependencies:

```bash
make build
```

This creates an image named `intent-mpc` with:
- ROS Noetic (Ubuntu 20.04)
- Gazebo 11
- All Intent-MPC packages
- DYNUS obstacles
- Required dependencies (Eigen, PCL, OpenCV, OsqpEigen, etc.)

Build time: ~5-10 minutes (depending on system and network speed)

### Rebuild After Code Changes

After modifying source code:

```bash
make build
```

The Docker build system automatically detects changes and rebuilds only modified packages.

### Clean Build (if needed)

To force a complete rebuild from scratch:

```bash
# Remove existing image
docker rmi intent-mpc

# Clear Docker build cache
docker system prune -a

# Rebuild
make build
```

## Running Simulations

### Dynamic Obstacle Simulation (DYNUS) - RECOMMENDED

Run the full MPC navigation with DYNUS obstacles using automated tmuxp launch:

```bash
make run-dynamic-gazebo NUM_OBSTACLES=200 DYNAMIC_RATIO=0.7 SEED=42
```

**Parameters:**
- `NUM_OBSTACLES`: Total number of obstacles (default: 200)
  - 70% dynamic (moving along trefoil trajectories)
  - 30% static (vertical pillars: 0.4×0.4×4.0m, horizontal walls: 0.4×4.0×0.4m)
- `DYNAMIC_RATIO`: Percentage of dynamic vs static (default: 0.7)
- `SEED`: Random seed for reproducible obstacle placement (default: 0)

**What launches:**
- Gazebo simulator (headless for performance)
- DYNUS obstacle generator
- Quadrotor model (no camera - uses fake detector)
- MPC navigation node
- Tracking controller
- RViz visualization
- All in a tmux session with 4 panes

**Mission:**
- Straight-line trajectory from (0, 0, 3) to (105, 0, 3)
- Drone autonomously avoids all obstacles
- Speed: 2.0 m/s desired velocity

### Manual Launch (Alternative)

For debugging or custom setups, you can launch manually:

```bash
# Launch interactive shell
make shell

# Inside container - Terminal 1: Start Gazebo + DYNUS
roslaunch uav_simulator start_dynus.launch num_obstacles:=200

# Inside container - Terminal 2: Start navigation (after 8s)
sleep 8
rostopic pub -1 /CERLAB/quadcopter/takeoff std_msgs/Empty
roslaunch autonomous_flight intent_mpc_dynus_nav.launch
```

### Original Demo (Circular Trajectory)

Run the original circular demo without DYNUS:

```bash
make run-original-demo
```

### Interactive Shell (Advanced)

For debugging or custom configurations, launch a shell inside the container:

```bash
make shell

# Inside container:
roslaunch uav_simulator start_dynus.launch num_obstacles:=50
# In another pane (Ctrl+b c):
roslaunch autonomous_flight intent_mpc_dynus_nav.launch
```

**Note:** For normal use, prefer `make run-dynamic-gazebo` which handles the timing and setup automatically.

### Tmux Session Layout

The simulation runs in tmux with 4 panes:

```
┌─────────────────────┬─────────────────────┐
│  Gazebo             │  Navigation         │
│  (simulator)        │  (MPC + controller) │
├─────────────────────┼─────────────────────┤
│  RViz               │  Command Pane       │
│  (visualization)    │  (manual control)   │
└─────────────────────┴─────────────────────┘
```

**Tmux Controls:**
- `Ctrl+b` then arrow keys: Navigate between panes
- `Ctrl+b` then `[`: Scroll mode (press `q` to exit)
- `Ctrl+b` then `c`: Create new window
- `Ctrl+b` then `d`: Detach from session
- `tmux attach -t dynus_sim`: Reattach to session
- `Ctrl+b` then `:kill-session`: Kill session

### Stopping Simulations

```bash
# If running manually in shell:
# Press Ctrl+C in each terminal pane

# If running with tmuxp:
make stop

# Or manually kill tmux sessions
tmux kill-session -t dynus_sim

# From within tmux
Ctrl+b, then type `:kill-session` and press Enter
```

### Monitoring During Simulation

```bash
# Attach to running simulation
tmux attach -t dynus_sim

# In a separate terminal, monitor topics:
docker exec -it $(docker ps -q --filter ancestor=intent-mpc) bash
rostopic hz /dynus/model_states
rostopic hz /mpc_planner/dynamic_obstacles
rostopic echo /CERLAB/quadcopter/pose --noarr
```

## Development Workflow

### Project Structure

```
Intent-MPC/
├── autonomous_flight/         # Main navigation node (MPC integration)
│   ├── include/autonomous_flight/
│   │   ├── flightBase.cpp    # Base flight functionality
│   │   └── mpcNavigation.cpp # MPC navigation implementation
│   └── cfg/mpc_navigation/   # Configuration files
├── trajectory_planner/        # Planning algorithms
│   ├── include/trajectory_planner/
│   │   ├── mpcPlanner.cpp    # MPC implementation
│   │   └── clustering/       # Static obstacle clustering
│   └── cfg/
├── onboard_detector/          # Obstacle detection
│   └── include/onboard_detector/
│       └── fakeDetector.cpp  # Ground truth detector
├── dynamic_predictor/         # Intent-aware prediction
├── tracking_controller/       # Low-level control
├── dynus_obstacles_ros1/      # DYNUS obstacle generation
│   ├── src/
│   │   ├── dynus_obstacles_node.cpp
│   │   └── fake_odom_node.cpp
│   └── launch/
├── uav_simulator/             # Gazebo simulation
│   ├── launch/
│   │   ├── start_dynus.launch
│   │   └── start.launch
│   └── urdf/
└── docker/                    # Docker setup
    ├── Dockerfile
    ├── Makefile
    └── README.md (this file)
```

### Typical Development Cycle

#### 1. Edit Code

Edit source files in `/home/kkondo/code/ip-mpc_ws/Intent-MPC/`:

**Example: Modify MPC parameters**
```bash
vim autonomous_flight/cfg/mpc_navigation/planner_param.yaml
```

**Example: Fix obstacle detection**
```bash
vim onboard_detector/include/onboard_detector/fakeDetector.cpp
```

#### 2. Build Changes

```bash
cd docker/
make build
```

Build output shows which packages were rebuilt.

#### 3. Test Changes

```bash
make run-dynamic-gazebo NUM_OBSTACLES=200 DYNAMIC_RATIO=0.7 SEED=42
```

Observe behavior in RViz and check console output.

**Alternative:** For manual debugging, use `make shell` and launch components separately.

#### 4. Debug Issues

**View logs in real-time:**
```bash
# Attach to tmux session
tmux attach -t dynus_sim

# Navigate to navigation pane (Ctrl+b, arrow keys)
# Scroll through output (Ctrl+b, [, then arrow keys, q to exit)
```

**Check specific topics:**
```bash
# In separate terminal
docker exec -it $(docker ps -q --filter ancestor=intent-mpc) bash

# Inside container:
rostopic echo /mpcNavigation/mpc_trajectory --noarr -n 1
rostopic hz /mpc_planner/dynamic_obstacles
rostopic list | grep obstacle
```

**Interactive debugging:**
```bash
make shell

# Inside container, run nodes manually:
roslaunch uav_simulator start_dynus.launch num_obstacles:=50

# In another pane:
rosrun autonomous_flight mpc_navigation_node
```

#### 5. Commit Working Changes

See [Git Workflow](#git-workflow) section below.

## Git Workflow

### Before You Start

Check your current status:

```bash
git status
git branch
git log --oneline -5
```

### Committing Changes

#### Step 1: Review Changes

```bash
# See what files changed
git status

# View detailed changes
git diff

# View changes in specific file
git diff Intent-MPC/onboard_detector/include/onboard_detector/fakeDetector.cpp
```

#### Step 2: Stage Files

**Option A: Stage specific files (RECOMMENDED)**
```bash
git add Intent-MPC/autonomous_flight/include/autonomous_flight/mpcNavigation.cpp
git add Intent-MPC/onboard_detector/include/onboard_detector/fakeDetector.cpp
git add Intent-MPC/trajectory_planner/include/trajectory_planner/mpcPlanner.cpp
```

**Option B: Stage all changes (use with caution)**
```bash
# Check what will be staged first
git status

# Stage everything
git add .
```

**Important:** Never stage:
- Build artifacts (`build/`, `devel/`, `install/`, `*.o`, `*.so`)
- Large binary files
- Sensitive data (credentials, API keys)
- Temporary files (`*~`, `*.swp`, `.DS_Store`)

#### Step 3: Commit with Descriptive Message

```bash
git commit -m "Fix obstacle size parsing for tall pillars and long walls

- Corrected substring indices in fakeDetector.cpp for obstacle name parsing
- Format: obstacle_dXXX_YYY_ZZZ where XXX/YYY/ZZZ are 3-digit cm values
- Fixed vertical pillars (0.4×0.4×4.0m) showing wrong z-axis size
- Fixed horizontal walls (0.4×4.0×0.4m) showing wrong y-axis size
- Changed substr indices from (11,15,19) to (10,14,18)
- All obstacle bounding boxes now accurately reflect true dimensions

Tested with 200 obstacles, verified tall pillars and wide walls display correctly.

Co-Authored-By: Claude Sonnet 4.5 <noreply@anthropic.com>"
```

**Commit Message Guidelines:**
- **First line**: Short summary (50-72 chars), imperative mood ("Fix", not "Fixed")
- **Blank line**: Separates summary from body
- **Body**: Detailed explanation of what changed and why
  - Use bullet points for multiple changes
  - Include context (which files, what problem it solves)
  - Mention any testing done
- **Footer**: Add `Co-Authored-By:` if pair programming

#### Step 4: Verify Commit

```bash
# View commit history
git log --oneline -5

# View last commit details
git show HEAD

# View commit graph
git log --oneline --graph --all -10
```

### Pushing Changes

#### Push to Remote Repository

```bash
# Check remote URL
git remote -v

# Pull latest changes first (always do this!)
git pull origin main

# If conflicts occur, resolve them:
git status  # See conflicted files
# Edit files to resolve conflicts
git add <resolved-files>
git commit -m "Resolve merge conflicts"

# Push changes
git push origin main
```

#### Working with Branches

**Create feature branch for new work:**
```bash
# Create and switch to new branch
git checkout -b feature/dynus-integration

# Make changes and commit
git add <files>
git commit -m "Add DYNUS obstacle generation"

# Push branch to remote
git push -u origin feature/dynus-integration
```

**Merge branch back to main:**
```bash
# Switch to main
git checkout main

# Pull latest
git pull origin main

# Merge feature branch
git merge feature/dynus-integration

# Push merged result
git push origin main

# Delete feature branch (optional)
git branch -d feature/dynus-integration
git push origin --delete feature/dynus-integration
```

### Creating Pull Requests

If working with a fork for upstream contribution:

```bash
# 1. Fork the repository on GitHub
# Click "Fork" button on https://github.com/original-owner/Intent-MPC

# 2. Clone your fork
git clone https://github.com/YOUR_USERNAME/Intent-MPC.git
cd Intent-MPC

# 3. Add upstream remote
git remote add upstream https://github.com/original-owner/Intent-MPC.git

# 4. Create feature branch
git checkout -b feature/amazing-feature

# 5. Make changes and commit
git add <files>
git commit -m "Add amazing feature"

# 6. Push to your fork
git push -u origin feature/amazing-feature

# 7. Create Pull Request on GitHub
# Go to https://github.com/YOUR_USERNAME/Intent-MPC
# Click "Pull Request" → "New Pull Request"
# Select your feature branch → Create PR
```

### Useful Git Commands

```bash
# View commit history
git log --oneline --graph --all

# Undo last commit (keep changes in working directory)
git reset --soft HEAD~1

# Undo last commit (discard changes - DANGEROUS)
git reset --hard HEAD~1

# Discard uncommitted changes in file
git checkout -- <file>

# Discard all uncommitted changes (DANGEROUS)
git reset --hard HEAD

# Stash changes temporarily
git stash
git stash list
git stash pop

# View changes in staged files
git diff --cached

# Amend last commit message
git commit --amend -m "New message"

# Amend last commit with new changes
git add <forgotten-file>
git commit --amend --no-edit

# List all branches
git branch -a

# Delete local branch
git branch -d feature/old-branch

# Update remote URL
git remote set-url origin <new-url>

# Cherry-pick specific commit
git cherry-pick <commit-hash>

# View file history
git log --follow -- <file>

# Blame (who changed what)
git blame <file>
```

### Git Best Practices

1. **Commit often**: Small, focused commits are easier to review and revert
2. **Pull before push**: Always sync with remote before pushing
3. **Descriptive messages**: Future you will thank you
4. **Don't commit generated files**: Use `.gitignore`
5. **Branch for features**: Keep main stable
6. **Review before committing**: Use `git diff` to check what you're committing
7. **Test before pushing**: Ensure code builds and tests pass

### Git Troubleshooting

**Problem:** Accidentally committed wrong files
```bash
# Undo last commit, keep changes
git reset --soft HEAD~1

# Restage only correct files
git add <correct-files>
git commit -m "Correct commit message"
```

**Problem:** Merge conflicts
```bash
# View conflicted files
git status

# Open conflicted files and resolve (look for <<<<<<< markers)
# After resolving:
git add <resolved-files>
git commit -m "Resolve merge conflicts"
```

**Problem:** Pushed wrong commit
```bash
# If no one else pulled yet (DANGEROUS):
git reset --hard HEAD~1
git push --force origin main

# Better: revert the commit (creates new commit)
git revert HEAD
git push origin main
```

**Problem:** Want to see what changed in a commit
```bash
git show <commit-hash>
git diff <commit-hash>^ <commit-hash>
```

## Configuration

### MPC Parameters

Edit `autonomous_flight/cfg/mpc_navigation/planner_param.yaml`:

```yaml
# Planning horizon
mpc_planner/horizon: 25  # timesteps (25 * 0.1s = 2.5s lookahead)

# Safety distances (how close to get to obstacles)
mpc_planner/static_safety_dist: 0.5   # meters
mpc_planner/dynamic_safety_dist: 0.4  # meters

# Constraint slack (higher = less conservative avoidance)
mpc_planner/static_constraint_slack_ratio: 0.05
mpc_planner/dynamic_constraint_slack_ratio: 0.3

# Local cloud region (should match planning horizon distance)
# At 2.0 m/s, 2.5s horizon = 5m, so use 7m with buffer
mpc_planner/local_cloud_region_x: 7.0  # forward (meters)
mpc_planner/local_cloud_region_y: 4.0  # lateral (meters)
```

**For more aggressive avoidance:**
```yaml
mpc_planner/static_safety_dist: 0.35
mpc_planner/dynamic_safety_dist: 0.30
mpc_planner/static_constraint_slack_ratio: 0.1
mpc_planner/dynamic_constraint_slack_ratio: 0.4
```

See `TUNING_GUIDE.md` for detailed parameter explanations.

### Flight Parameters

Edit `autonomous_flight/cfg/mpc_navigation/flight_base.yaml`:

```yaml
simulation: true
use_fake_detector: true  # Ground truth detection
use_predictor: true      # Intent-aware prediction

# Speeds
desired_velocity: 2.0       # m/s (increased from 1.5 for faster navigation)
desired_acceleration: 2.0   # m/s^2 (increased from 1.5)
desired_angular_velocity: 0.5  # rad/s

# Altitude
takeoff_height: 3.0  # meters (matches spawn height)

# Goal trajectory
use_predefined_goal: true
predefined_goal_directory: "/cfg/mpc_navigation/ref_trajectory_straight_line.txt"
execute_path_times: 1  # execute once (was 10 for circular demo)
```

### DYNUS Obstacle Parameters

Edit `uav_simulator/launch/start_dynus.launch`:

```xml
<!-- Number of obstacles -->
<arg name="num_obstacles" default="200"/>

<!-- Ratio of dynamic vs static (0.7 = 70% dynamic, 30% static) -->
<arg name="dynamic_ratio" default="0.7"/>

<!-- Spatial bounds (corridor: 100m long × 14m wide × 4m tall) -->
<arg name="x_min" default="0.0"/>
<arg name="x_max" default="100.0"/>
<arg name="y_min" default="-7.0"/>
<arg name="y_max" default="7.0"/>
<arg name="z_min" default="0.5"/>
<arg name="z_max" default="4.5"/>

<!-- Movement speed for dynamic obstacles -->
<!-- Higher values = slower movement (inverse relationship) -->
<arg name="slower_min" default="4.0"/>
<arg name="slower_max" default="6.0"/>
```

### Fake Detector Configuration

Edit `autonomous_flight/cfg/mpc_navigation/fake_detector_param.yaml`:

```yaml
# Topic configuration
model_states_topic: "/dynus/model_states"  # Changed from /gazebo/model_states
odom_topic: "/CERLAB/quadcopter/odom"

# Target obstacles to track (prefix matching)
target_obstacle: ["obstacle"]

# History size for trajectory prediction
history_size: 10
```

### RViz Configuration

The RViz config is at `remote_control/rviz/mpc_navigation.rviz`. Key displays:

- **Reference Trajectory**: Green line (desired path)
- **MPC Trajectory**: Blue line (planned path with obstacle avoidance)
- **Executed Trajectory**: Red line (actual flown path)
- **Dynamic Obstacles**: Blue/red boxes (from fake detector)
- **DYNUS Markers**: Ground truth obstacle visualization
- **Robot Model**: Quadrotor visualization

## Troubleshooting

### Build Issues

**Problem:** Docker build fails with "out of memory"
```bash
# Solution: Increase Docker memory limit
# Docker Desktop → Settings → Resources → Memory → Increase to 8GB
```

**Problem:** `catkin_make` fails during build
```bash
# Solution 1: Clean and rebuild
docker rmi intent-mpc
docker system prune -a
make build

# Solution 2: Check for syntax errors in modified files
# Review git diff output for any obvious issues
```

**Problem:** "Package 'acl_sim' not found"
```bash
# Solution: Ensure submodules are initialized
cd /home/kkondo/code/ip-mpc_ws
git submodule update --init --recursive
make build
```

### Runtime Issues

**Problem:** "Cannot connect to X server"
```bash
# Solution: Allow Docker to access X11
xhost +local:docker

# On Linux, you may need:
xhost +local:root

# Verify DISPLAY is set
echo $DISPLAY
```

**Problem:** Gazebo crashes or freezes
```bash
# Solution 1: Already running headless (gui:=false in start_dynus.launch)

# Solution 2: Increase Docker resources
# Docker Desktop → Settings → Resources
# CPU: 4+ cores, Memory: 8GB

# Solution 3: Use software rendering
export LIBGL_ALWAYS_SOFTWARE=1
```

**Problem:** MPC not avoiding obstacles
```bash
# Check 1: Verify obstacles are being detected
docker exec -it $(docker ps -q) bash
rostopic hz /dynus/model_states           # Should be ~50 Hz
rostopic hz /mpc_planner/dynamic_obstacles  # Should be ~20 Hz

# Check 2: Verify fake_detector configuration
cat /root/ip-mpc_ws/src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/fake_detector_param.yaml
# Ensure model_states_topic is "/dynus/model_states"

# Check 3: View detected obstacles
rostopic echo /mpc_planner/dynamic_obstacles --noarr -n 5

# Check 4: Monitor MPC output
# Attach to tmux and watch navigation pane output
tmux attach -t dynus_sim
```

**Problem:** Drone not taking off
```bash
# Check if takeoff command was sent (should auto-trigger at t+8s)
rostopic echo /CERLAB/quadcopter/takeoff

# Manually trigger takeoff
docker exec -it $(docker ps -q) bash
rostopic pub -1 /CERLAB/quadcopter/takeoff std_msgs/Empty

# Check quadrotor position
rostopic echo /CERLAB/quadcopter/pose --noarr -n 1
# Should show z ≈ 3.0 after takeoff
```

**Problem:** Obstacle sizes look wrong
```bash
# Check obstacle names in model_states
rostopic echo /dynus/model_states --noarr -n 1

# Names should be format: obstacle_dXXX_YYY_ZZZ
# Example: obstacle_d040_400_040 = 0.4m × 4.0m × 0.4m

# If sizes still wrong, verify parsing in fakeDetector.cpp:
# substr(10,3), substr(14,3), substr(18,3) for positions 0-based
```

### Docker Issues

**Problem:** Container won't start
```bash
# Check Docker daemon is running
docker ps

# Check for port conflicts
lsof -ti:11311 | xargs kill -9  # Kill any process on ROS master port

# View container logs
docker ps -a  # Get container ID
docker logs <container-id>

# Remove stale containers
docker container prune
```

**Problem:** "Port already in use"
```bash
# Stop conflicting simulation
make stop

# Or manually kill ROS master
lsof -ti:11311 | xargs kill -9
```

**Problem:** Docker build uses too much disk space
```bash
# Clean up Docker system
docker system df  # Check disk usage
docker system prune -a  # Clean everything (CAUTION)

# Or be more selective
docker image prune  # Remove unused images
docker container prune  # Remove stopped containers
```

**Problem:** GPU not available in container
```bash
# Check nvidia-docker installation
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# If fails, install nvidia-docker2:
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### Performance Issues

**Problem:** Simulation runs slowly
```bash
# Solution 1: Reduce number of obstacles
# When launching, use: roslaunch uav_simulator start_dynus.launch num_obstacles:=50

# Solution 2: Disable RViz visualization
# Don't launch RViz, or comment it out in launch files

# Solution 3: Check CPU/memory usage
docker stats

# Solution 4: Increase Docker resources
# Docker Desktop → Settings → Resources → Increase limits
```

**Problem:** High CPU usage
```bash
# Check which processes are consuming CPU
docker exec -it $(docker ps -q) bash
top

# Common culprits:
# - gazebo (physics simulation)
# - rviz (visualization)
# - mpc_navigation_node (trajectory planning)

# Reduce load by lowering obstacle count or disabling visualization
```

## Additional Resources

### Key ROS Topics

Monitor these topics during simulation:

```bash
# Obstacle information
/dynus/model_states                      # Ground truth obstacles (50 Hz)
/dynus_obstacles_markers                 # Visualization markers
/mpc_planner/dynamic_obstacles           # Detected obstacles (visualization)

# Trajectories
/mpcNavigation/reference_trajectory      # Desired path (green)
/mpcNavigation/mpc_trajectory           # Planned path (blue)
/mpcNavigation/trajectory               # Executed path (red)

# Robot state
/CERLAB/quadcopter/odom                 # Odometry (30 Hz)
/CERLAB/quadcopter/pose                 # Position (30 Hz)
/CERLAB/quadcopter/vel                  # Velocity (30 Hz)
/cmd_vel                                # Control commands

# Control
/CERLAB/quadcopter/takeoff              # Takeoff trigger
/CERLAB/quadcopter/cmd_vel              # Velocity commands
```

### Useful Commands

```bash
# List all topics
rostopic list

# Check topic frequency
rostopic hz <topic_name>

# Echo topic content
rostopic echo <topic_name>

# Show topic info
rostopic info <topic_name>

# Monitor TF tree
rosrun tf view_frames
evince frames.pdf

# Record data
rosbag record -a -O simulation_data.bag

# Play recorded data
rosbag play simulation_data.bag
```

### Parameter Tuning

See the comprehensive tuning guide:
```bash
cat /root/ip-mpc_ws/src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/TUNING_GUIDE.md
```

Or view in the repository at:
`autonomous_flight/cfg/mpc_navigation/TUNING_GUIDE.md`

### Documentation

- **ROS Noetic**: https://wiki.ros.org/noetic
- **Gazebo**: https://gazebosim.org/
- **Docker**: https://docs.docker.com/
- **DYNUS**: [Original DYNUS repository]
- **Intent-MPC**: [Original paper/repository]

## Makefile Targets

Available commands:

```bash
make build              # Build Docker image
make run-dynamic-gazebo # Run DYNUS simulation (RECOMMENDED)
                        # Usage: make run-dynamic-gazebo NUM_OBSTACLES=200 DYNAMIC_RATIO=0.7 SEED=42
make stop               # Stop all tmux simulation sessions
make shell              # Open bash shell in container (for manual launch/debugging)
make run-demo           # Run original circular demo
make run                # Run container (interactive)
make clean              # Remove stopped containers
make help               # Show help message
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes with descriptive messages
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

[Add license information here]

## Acknowledgments

- Intent-MPC original authors
- DYNUS obstacle generation
- ROS and Gazebo communities
- All contributors

---

**Last Updated**: 2026-02-05
**Maintainer**: Kota Kondo (kkondo@mit.edu)
**Docker Environment**: ROS Noetic (Ubuntu 20.04) + Gazebo 11
