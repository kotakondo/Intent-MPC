# Makefile for Intent-MPC Docker environment
# Based on FASTER docker pattern

.PHONY: build build-clone run run-gazebo run-demo run-dynamic-gazebo run-dev run-gazebo-dev run-benchmark run-benchmark-sweep run-full-sweep analyze-benchmark analyze-benchmark-sweep shell stop clean help

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
# Usage: make run-benchmark [DIFFICULTY=hard] [NUM_TRIALS=20] [MAX_VEL=5.0] [MAX_ACC=20.0] [SEED_START=0] [VISUALIZE=1]
# Internal: OUTPUT_SUBDIR sets a subdirectory under data/ (used by run-full-sweep)
run-benchmark:
	@echo "Running Intent-MPC benchmark (DYNUS-compatible configuration):"
	@echo "  DIFFICULTY          = $(if $(DIFFICULTY),$(DIFFICULTY),hard (default))"
	@echo "  NUM_TRIALS          = $(if $(NUM_TRIALS),$(NUM_TRIALS),20 (default))"
	@echo "  MAX_VEL             = $(if $(MAX_VEL),$(MAX_VEL),5.0 (default)) m/s"
	@echo "  MAX_ACC             = $(if $(MAX_ACC),$(MAX_ACC),20.0 (default)) m/s^2"
	@echo "  DYNAMIC_RATIO       = $(if $(DYNAMIC_RATIO),$(DYNAMIC_RATIO),0.65 (default, DYNUS match))"
	@echo "  SEED_START          = $(if $(SEED_START),$(SEED_START),0 (default))"
	@echo "  VISUALIZE           = $(if $(VISUALIZE),$(VISUALIZE) (RViz enabled),0 (headless))"
	@echo "  OUTPUT_SUBDIR       = $(if $(OUTPUT_SUBDIR),$(OUTPUT_SUBDIR),(data root))"
	@echo ""
	@echo "Data will be saved to: Intent-MPC/data/$(if $(OUTPUT_SUBDIR),$(OUTPUT_SUBDIR)/,)"
	@mkdir -p ../data$(if $(OUTPUT_SUBDIR),/$(OUTPUT_SUBDIR),)
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
		/bin/bash -c "cd /root/ip-mpc_ws && source devel/setup.bash && source src/Intent-MPC/uav_simulator/gazeboSetup.bash && python3 src/Intent-MPC/scripts/run_mpc_benchmark.py --num-trials $(if $(NUM_TRIALS),$(NUM_TRIALS),20) --difficulty $(if $(DIFFICULTY),$(DIFFICULTY),hard) --max-vel $(if $(MAX_VEL),$(MAX_VEL),5.0) --max-acc $(if $(MAX_ACC),$(MAX_ACC),20.0) --dynamic-ratio $(if $(DYNAMIC_RATIO),$(DYNAMIC_RATIO),0.65) --seed-start $(if $(SEED_START),$(SEED_START),0) $(if $(OUTPUT_SUBDIR),--output-dir /root/ip-mpc_ws/src/Intent-MPC/data/$(OUTPUT_SUBDIR),) $(if $(VISUALIZE),--visualize,)"

# Run benchmark with RViz visualization (convenience target)
# Usage: make run-benchmark-viz [NUM_TRIALS=1] [NUM_OBSTACLES=50]
run-benchmark-viz:
	$(MAKE) run-benchmark VISUALIZE=1 NUM_TRIALS=$(if $(NUM_TRIALS),$(NUM_TRIALS),1) NUM_OBSTACLES=$(if $(NUM_OBSTACLES),$(NUM_OBSTACLES),50)

# Run benchmark sweep over difficulty levels (single vel/acc config)
# Usage: make run-benchmark-sweep [NUM_TRIALS=20] [MAX_VEL=5.0] [MAX_ACC=20.0] [SEED_START=0]
# Default: easy + medium only (assumes hard already exists)
DIFFICULTY_LEVELS ?= easy medium
run-benchmark-sweep:
	@echo "Running I-MPC benchmark sweep over difficulty levels: $(DIFFICULTY_LEVELS)"
	@echo "  MAX_VEL       = $(if $(MAX_VEL),$(MAX_VEL),5.0 (default)) m/s"
	@echo "  MAX_ACC       = $(if $(MAX_ACC),$(MAX_ACC),20.0 (default)) m/s^2"
	@echo "  NUM_TRIALS    = $(if $(NUM_TRIALS),$(NUM_TRIALS),20 (default))"
	@echo ""
	@for diff in $(DIFFICULTY_LEVELS); do \
		echo ""; \
		echo "========================================"; \
		echo "  SWEEP: difficulty=$$diff"; \
		echo "========================================"; \
		$(MAKE) run-benchmark DIFFICULTY=$$diff \
			NUM_TRIALS=$(if $(NUM_TRIALS),$(NUM_TRIALS),20) \
			MAX_VEL=$(if $(MAX_VEL),$(MAX_VEL),5.0) \
			MAX_ACC=$(if $(MAX_ACC),$(MAX_ACC),20.0) \
			DYNAMIC_RATIO=$(if $(DYNAMIC_RATIO),$(DYNAMIC_RATIO),0.65) \
			SEED_START=$(if $(SEED_START),$(SEED_START),0) \
			$(if $(OUTPUT_SUBDIR),OUTPUT_SUBDIR=$(OUTPUT_SUBDIR),); \
	done
	@echo ""
	@echo "Sweep complete. Benchmark directories:"
	@ls -d ../data/$(if $(OUTPUT_SUBDIR),$(OUTPUT_SUBDIR)/,)*_benchmark_* 2>/dev/null || echo "  (none found)"

# Full sweep: all velocity/acceleration configs x all difficulty levels
# Usage: make run-full-sweep [CONFIGS="1.5:1.5 3:3 5:20"] [NUM_TRIALS=10]
#
# CONFIGS is a space-separated list of vel:acc pairs.
# Each config produces a directory: data/max_vel_<vel>_acc_<acc>/
# containing {easy,medium,hard}_benchmark_<timestamp>/ subdirectories.
#
# Example: make run-full-sweep CONFIGS="1.5:1.5 3:3" NUM_TRIALS=5
CONFIGS ?= 1.5:1.5 3:3 5:20
run-full-sweep:
	@echo "========================================"
	@echo "  FULL I-MPC BENCHMARK SWEEP"
	@echo "========================================"
	@echo "  CONFIGS     = $(CONFIGS)"
	@echo "  DIFFICULTIES = easy medium hard"
	@echo "  NUM_TRIALS  = $(if $(NUM_TRIALS),$(NUM_TRIALS),10 (default))"
	@echo "  SEED_START  = $(if $(SEED_START),$(SEED_START),0 (default))"
	@echo ""
	@echo "This will run $(words $(CONFIGS)) configs x 3 difficulties = $(shell echo $$(($(words $(CONFIGS)) * 3))) benchmark sets."
	@echo ""
	@for config in $(CONFIGS); do \
		vel=$$(echo $$config | cut -d: -f1); \
		acc=$$(echo $$config | cut -d: -f2); \
		echo ""; \
		echo "########################################"; \
		echo "# CONFIG: max_vel=$$vel, max_acc=$$acc"; \
		echo "########################################"; \
		$(MAKE) run-benchmark-sweep \
			DIFFICULTY_LEVELS="easy medium hard" \
			MAX_VEL=$$vel \
			MAX_ACC=$$acc \
			NUM_TRIALS=$(if $(NUM_TRIALS),$(NUM_TRIALS),10) \
			SEED_START=$(if $(SEED_START),$(SEED_START),0) \
			OUTPUT_SUBDIR=max_vel_$${vel}_acc_$${acc} \
			$(if $(VISUALIZE),VISUALIZE=1,); \
	done
	@echo ""
	@echo "========================================"
	@echo "  FULL SWEEP COMPLETE"
	@echo "========================================"
	@echo ""
	@echo "Config directories:"
	@ls -d ../data/max_vel_*_acc_* 2>/dev/null || echo "  (none found)"
	@echo ""
	@echo "To analyze all results and update LaTeX table:"
	@echo "  make analyze-benchmark-sweep"

# Analyze benchmark data and generate LaTeX table
# Usage: make analyze-benchmark DATA_DIR=benchmark_wa10.0_20260205_143512
analyze-benchmark:
	@if [ -z "$(DATA_DIR)" ]; then \
		echo "ERROR: DATA_DIR is required"; \
		echo "Usage: make analyze-benchmark DATA_DIR=benchmark_wa10.0_TIMESTAMP"; \
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
		/bin/bash -c "pip install pandas rosbags -q && cd /root/ip-mpc_ws && source devel/setup.bash && python3 src/Intent-MPC/scripts/analyze_benchmark.py --data-dir src/Intent-MPC/data/$(DATA_DIR)"

# Analyze all sweep results together (multi-config: max_vel_X_acc_Y directories)
# Usage: make analyze-benchmark-sweep
# This finds all max_vel_*_acc_* config directories and analyzes them together.
# Each config dir should contain {easy,medium,hard}_benchmark_* subdirectories.
analyze-benchmark-sweep:
	@echo "Analyzing all config directories (max_vel_*_acc_*)..."
	docker run -it \
		--gpus all \
		--privileged \
		--volume=$(shell cd ../.. && pwd):/root/ip-mpc_ws/src:rw \
		--volume=/home/kkondo/paper_writing:/home/kkondo/paper_writing:rw \
		--rm \
		$(IMAGE_NAME) \
		/bin/bash -c 'pip install pandas rosbags -q && cd /root/ip-mpc_ws && source devel/setup.bash && config_dirs=$$(ls -d src/Intent-MPC/data/max_vel_*_acc_* 2>/dev/null) && if [ -z "$$config_dirs" ]; then echo "ERROR: No max_vel_*_acc_* directories found"; exit 1; fi && echo "Found config dirs: $$config_dirs" && python3 src/Intent-MPC/scripts/analyze_benchmark.py --config-dirs $$config_dirs --skip-collision-postprocess'

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
	@echo ""
	@echo "Benchmarking:"
	@echo "  run-benchmark        - Run benchmark for a single difficulty + config"
	@echo "                           make run-benchmark DIFFICULTY=hard MAX_VEL=3.0 MAX_ACC=3.0 NUM_TRIALS=10"
	@echo "  run-benchmark-sweep  - Sweep difficulty levels for a single vel/acc config"
	@echo "                           make run-benchmark-sweep DIFFICULTY_LEVELS='easy medium hard' MAX_VEL=3.0 MAX_ACC=3.0"
	@echo "  run-full-sweep       - Sweep ALL configs x ALL difficulties (RECOMMENDED)"
	@echo "                           make run-full-sweep CONFIGS='1.5:1.5 3:3 5:20' NUM_TRIALS=10"
	@echo "                           Data saved to: data/max_vel_<v>_acc_<a>/{easy,medium,hard}_benchmark_*/"
	@echo ""
	@echo "Analysis:"
	@echo "  analyze-benchmark-sweep - Analyze all max_vel_*_acc_* dirs, update LaTeX table"
	@echo "  analyze-benchmark       - Analyze single dir: DATA_DIR=max_vel_3_acc_3/hard_benchmark_..."
	@echo ""
	@echo "Other:"
	@echo "  shell            - Run the container and start bash"
	@echo "  stop             - Stop all running simulation sessions"
	@echo "  clean            - Remove stopped containers"
	@echo "  help             - Display this help message"
	@echo ""
	@echo "Before running, ensure X server access:"
	@echo "  xhost +local:docker"
	@echo ""
	@echo "Quick start - Full benchmark:"
	@echo "  1. xhost +local:docker"
	@echo "  2. make build"
	@echo "  3. make run-full-sweep NUM_TRIALS=10"
	@echo "  4. make analyze-benchmark-sweep"
