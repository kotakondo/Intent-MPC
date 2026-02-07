#!/bin/bash
# Simple script to run Intent-MPC with DYNUS obstacles
# No Gazebo required - uses ground truth odometry

set -e

# Default parameters
NUM_OBSTACLES=10
DYNAMIC_RATIO=0.5
SEED=0

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --num-obstacles)
            NUM_OBSTACLES="$2"
            shift 2
            ;;
        --dynamic-ratio)
            DYNAMIC_RATIO="$2"
            shift 2
            ;;
        --seed)
            SEED="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --num-obstacles NUM   Number of total obstacles (default: 10)"
            echo "  --dynamic-ratio RATIO Ratio of dynamic obstacles 0.0-1.0 (default: 0.5)"
            echo "  --seed SEED           Random seed (default: 0)"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "=================================="
echo "Intent-MPC with DYNUS Obstacles"
echo "=================================="
echo "Parameters:"
echo "  Total obstacles: $NUM_OBSTACLES"
echo "  Dynamic ratio:   $DYNAMIC_RATIO"
echo "  Random seed:     $SEED"
echo "=================================="
echo ""

# Source workspace
source devel/setup.bash

# Launch simulation
roslaunch autonomous_flight intent_mpc_dynus_sim.launch \
    num_obstacles:=$NUM_OBSTACLES \
    dynamic_ratio:=$DYNAMIC_RATIO \
    seed:=$SEED
