# Makefile for Intent-MPC Docker environment
# Based on FASTER docker pattern

.PHONY: build build-clone run run-gazebo run-demo run-dynamic-gazebo run-dev run-gazebo-dev run-benchmark shell stop clean help

# Image name
IMAGE_NAME := intent-mpc

# Build the Docker image (standard incremental build)
build:
	cd ../.. && docker build -f Intent-MPC/docker/Dockerfile -t $(IMAGE_NAME) .

# Build with cache-busting to get latest code from git repos
build-clone:
	cd ../.. && docker build -f Intent-MPC/docker/Dockerfile --build-arg DUMMY=`date +%s` -t $(IMAGE_NAME) .

# Run the container with GUI support
run:
	docker run -it \
		--gpus all \
		--env="NVIDIA_DRIVER_CAPABILITIES=all" \
		--privileged \
		-e DISPLAY=${DISPLAY} \
		--network=host \
		--env="QT_X11_NO_MITSHM=1" \
		--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
		--rm \
		$(IMAGE_NAME)

# Run the container and explicitly start bash (with host workspace mounted)
shell:
	docker run -it \
		--gpus all \
		--env="NVIDIA_DRIVER_CAPABILITIES=all" \
		--privileged \
		-e DISPLAY=${DISPLAY} \
		--network=host \
		--env="QT_X11_NO_MITSHM=1" \
		--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw \
		--rm \
		$(IMAGE_NAME) \
		/bin/bash

# Run the container and automatically launch ACL hard_forest simulation
run-gazebo:
	docker run -it \
		--gpus all \
		--env="NVIDIA_DRIVER_CAPABILITIES=all" \
		--privileged \
		-e DISPLAY=${DISPLAY} \
		--network=host \
		--env="QT_X11_NO_MITSHM=1" \
		--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
		--rm \
		$(IMAGE_NAME) \
		/bin/bash -c "cd /root/ip-mpc_ws && source devel/setup.bash && tmuxp load /root/ip-mpc_ws/src/Intent-MPC/docker/acl_forest_sim.yml"

# Run the original Intent-MPC demo (circular trajectory)
run-demo:
	docker run -it \
		--gpus all \
		--env="NVIDIA_DRIVER_CAPABILITIES=all" \
		--privileged \
		-e DISPLAY=${DISPLAY} \
		--network=host \
		--env="QT_X11_NO_MITSHM=1" \
		--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw \
		--rm \
		$(IMAGE_NAME) \
		/bin/bash -c "cd /root/ip-mpc_ws && source devel/setup.bash && tmuxp load /root/ip-mpc_ws/src/Intent-MPC/docker/original_demo.yml"

# Run Intent-MPC with dynamic obstacles AND Gazebo physics simulation
# Usage: make run-dynamic-gazebo [NUM_OBSTACLES=200] [DYNAMIC_RATIO=0.7] [SEED=0]
run-dynamic-gazebo:
	@echo "Launching with Gazebo physics simulation (headless):"
	@echo "  NUM_OBSTACLES = $(if $(NUM_OBSTACLES),$(NUM_OBSTACLES),200 (default))"
	@echo "  DYNAMIC_RATIO = $(if $(DYNAMIC_RATIO),$(DYNAMIC_RATIO),0.7 (default))"
	@echo "  SEED          = $(if $(SEED),$(SEED),0 (default))"
	@echo ""
	@echo "Using split launch (like original demo): Gazebo first, then navigation"
	docker run -it \
		--gpus all \
		--env="NVIDIA_DRIVER_CAPABILITIES=all" \
		--privileged \
		-e DISPLAY=${DISPLAY} \
		-e NUM_OBSTACLES=$(if $(NUM_OBSTACLES),$(NUM_OBSTACLES),200) \
		-e DYNAMIC_RATIO=$(if $(DYNAMIC_RATIO),$(DYNAMIC_RATIO),0.7) \
		-e SEED=$(if $(SEED),$(SEED),0) \
		--network=host \
		--env="QT_X11_NO_MITSHM=1" \
		--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw \
		--rm \
		$(IMAGE_NAME) \
		/bin/bash -c "cd /root/ip-mpc_ws && source devel/setup.bash && tmuxp load /root/ip-mpc_ws/src/Intent-MPC/docker/dynus_sim.yml"

# Development mode: Run with host workspace mounted (changes reflect immediately)
run-dev:
	docker run -it \
		--gpus all \
		--env="NVIDIA_DRIVER_CAPABILITIES=all" \
		--privileged \
		-e DISPLAY=${DISPLAY} \
		--network=host \
		--env="QT_X11_NO_MITSHM=1" \
		--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw \
		--rm \
		$(IMAGE_NAME)

# Development mode: Auto-launch with mounted workspace
run-gazebo-dev:
	docker run -it \
		--gpus all \
		--env="NVIDIA_DRIVER_CAPABILITIES=all" \
		--privileged \
		-e DISPLAY=${DISPLAY} \
		--network=host \
		--env="QT_X11_NO_MITSHM=1" \
		--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw \
		--rm \
		$(IMAGE_NAME) \
		/bin/bash -c "cd /root/ip-mpc_ws && source devel/setup.bash && tmuxp load /root/ip-mpc_ws/src/Intent-MPC/docker/acl_forest_sim.yml"

# Run benchmark with data persistence (DYNUS-compatible parameters)
# Usage: make run-benchmark [NUM_TRIALS=20] [NUM_OBSTACLES=200] [DYNAMIC_RATIO=0.65] [SEED_START=0] [VISUALIZE=1]
run-benchmark:
	@echo "Running Intent-MPC benchmark (DYNUS-compatible configuration):"
	@echo "  NUM_TRIALS    = $(if $(NUM_TRIALS),$(NUM_TRIALS),20 (default))"
	@echo "  NUM_OBSTACLES = $(if $(NUM_OBSTACLES),$(NUM_OBSTACLES),200 (default))"
	@echo "  DYNAMIC_RATIO = $(if $(DYNAMIC_RATIO),$(DYNAMIC_RATIO),0.65 (default, DYNUS match))"
	@echo "  SEED_START    = $(if $(SEED_START),$(SEED_START),0 (default))"
	@echo "  VISUALIZE     = $(if $(VISUALIZE),$(VISUALIZE) (RViz enabled),0 (headless))"
	@echo ""
	@echo "Data will be saved to: Intent-MPC/data/"
	@mkdir -p ../data
	docker run -it \
		--gpus all \
		--env="NVIDIA_DRIVER_CAPABILITIES=all" \
		--privileged \
		-e DISPLAY=${DISPLAY} \
		--network=host \
		--env="QT_X11_NO_MITSHM=1" \
		--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
		--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw \
		--rm \
		$(IMAGE_NAME) \
		/bin/bash -c "cd /root/ip-mpc_ws && source devel/setup.bash && source src/Intent-MPC/uav_simulator/gazeboSetup.bash && python3 src/Intent-MPC/scripts/run_mpc_benchmark.py --num-trials $(if $(NUM_TRIALS),$(NUM_TRIALS),20) --num-obstacles $(if $(NUM_OBSTACLES),$(NUM_OBSTACLES),200) --dynamic-ratio $(if $(DYNAMIC_RATIO),$(DYNAMIC_RATIO),0.65) --seed-start $(if $(SEED_START),$(SEED_START),0) $(if $(VISUALIZE),--visualize,)"

# Run benchmark with RViz visualization (convenience target)
# Usage: make run-benchmark-viz [NUM_TRIALS=1] [NUM_OBSTACLES=50]
run-benchmark-viz:
	$(MAKE) run-benchmark VISUALIZE=1 NUM_TRIALS=$(if $(NUM_TRIALS),$(NUM_TRIALS),1) NUM_OBSTACLES=$(if $(NUM_OBSTACLES),$(NUM_OBSTACLES),50)

# Analyze benchmark data and generate LaTeX table
# Usage: make analyze-benchmark DATA_DIR=benchmark_20260205_143512
analyze-benchmark:
	@if [ -z "$(DATA_DIR)" ]; then \
		echo "ERROR: DATA_DIR is required"; \
		echo "Usage: make analyze-benchmark DATA_DIR=benchmark_TIMESTAMP"; \
		echo ""; \
		echo "Available benchmark directories:"; \
		ls -d ../data/benchmark_* 2>/dev/null || echo "  (none found)"; \
		exit 1; \
	fi
	@echo "Analyzing benchmark data: $(DATA_DIR)"
	docker run -it \
		--gpus all \
		--privileged \
		--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw \
		--volume=/home/kkondo/paper_writing:/home/kkondo/paper_writing:rw \
		--rm \
		$(IMAGE_NAME) \
		/bin/bash -c "pip install pandas -q && cd /root/ip-mpc_ws && source devel/setup.bash && python3 src/Intent-MPC/scripts/analyze_benchmark.py --data-dir src/Intent-MPC/data/$(DATA_DIR)"

# Stop running simulations
stop:
	-tmux kill-session -t dynus_sim 2>/dev/null || true
	-tmux kill-session -t intent_mpc_sim 2>/dev/null || true
	-tmux kill-session -t acl_forest_sim 2>/dev/null || true
	@echo "Stopped all simulation sessions"

# Remove stopped containers
clean:
	docker container prune -f

# Display help
help:
	@echo "Intent-MPC Docker Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  build            - Build the Docker image (incremental)"
	@echo "  build-clone      - Build with cache-busting for fresh git clone"
	@echo "  run              - Run the container with GUI support"
	@echo "  run-demo         - Run ORIGINAL demo (circular trajectory, generated_env.world)"
	@echo "  run-gazebo       - Run ACL hard_forest simulation"
	@echo "  run-dynamic-gazebo - Run with dynamic obstacles + Gazebo physics"
	@echo "                       Usage: make run-dynamic-gazebo [NUM_OBSTACLES=200] [DYNAMIC_RATIO=0.7] [SEED=0]"
	@echo "  run-dev          - Run with host workspace mounted (for development)"
	@echo "  run-gazebo-dev   - Auto-launch ACL simulation with mounted workspace"
	@echo "  run-benchmark    - Run benchmark trials with data persistence"
	@echo "                       Usage: make run-benchmark [NUM_TRIALS=20] [NUM_OBSTACLES=200] [DYNAMIC_RATIO=0.7] [SEED_START=0]"
	@echo "  shell            - Run the container and start bash"
	@echo "  stop             - Stop all running simulation sessions"
	@echo "  clean            - Remove stopped containers"
	@echo "  help             - Display this help message"
	@echo ""
	@echo "Before running, ensure X server access:"
	@echo "  xhost +local:docker"
	@echo ""
	@echo "Quick start - Test original demo:"
	@echo "  1. xhost +local:docker"
	@echo "  2. make build"
	@echo "  3. make run-demo"
	@echo ""
	@echo "Development workflow:"
	@echo "  - Build once: make build"
	@echo "  - Use run-demo to test baseline system"
	@echo "  - Use run-dev or run-gazebo-dev for development"
	@echo "  - Edit files on host, changes reflect immediately in container"
	@echo "  - Rebuild workspace inside container: cd /root/ip-mpc_ws && catkin_make"
	@echo "  - Only rebuild Docker image when dependencies change"
