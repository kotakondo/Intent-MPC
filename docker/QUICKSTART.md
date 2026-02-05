# Intent-MPC Docker Quick Start

## Setup (One-time)

```bash
# Allow Docker to access X server
xhost +local:docker

# Build the Docker image
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker
make build
```

## Run Simulation

```bash
# Start container
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker
make run

# Inside container - automated launch with TMUXP
tmuxp load /root/ip-mpc_ws/Intent-MPC/docker/intent_mpc_sim.yml
```

## Manual Launch (Alternative)

```bash
# Inside container
# Terminal 1:
roslaunch uav_simulator start.launch

# Terminal 2 (Ctrl+b c for new tmux pane):
roslaunch autonomous_flight intent_mpc_demo.launch
```

## TMUX Navigation

- `Ctrl+b` + arrow keys - Navigate panes
- `Ctrl+b c` - New window
- `Ctrl+b d` - Detach from session

## Rebuild After Code Changes

```bash
cd /home/kkondo/code/ip-mpc_ws/Intent-MPC/docker
make build
```
