#!/usr/bin/env python3
"""
Generate straight-line trajectory from (0,0,3) to (105,0,3) for hard_forest navigation.
Uses 0.1s timestep to match the original circular trajectory format.
"""
import numpy as np

def generate_straight_line_trajectory():
    start = np.array([0.0, 0.0, 3.0])
    end = np.array([105.0, 0.0, 3.0])

    # Match original trajectory format: 0.1 second timestep
    dt = 0.1  # seconds
    desired_velocity = 1.5  # m/s

    # Calculate distance and time
    distance = np.linalg.norm(end - start)
    total_time = distance / desired_velocity
    num_waypoints = int(total_time / dt) + 1

    output_file = 'ref_trajectory_straight_line.txt'

    with open(output_file, 'w') as f:
        for i in range(num_waypoints):
            t = i * dt
            # Linear interpolation
            alpha = min(t * desired_velocity / distance, 1.0)
            waypoint = start + alpha * (end - start)
            # Format: time x y z (4 columns, space-separated)
            f.write(f"{t:.1f} {waypoint[0]:.1f} {waypoint[1]:.1f} {waypoint[2]:.1f}\n")

    print(f"Generated trajectory with {num_waypoints} waypoints")
    print(f"Total distance: {distance:.1f} meters")
    print(f"Total time: {total_time:.1f} seconds at {desired_velocity} m/s")
    print(f"Timestep: {dt} seconds (matching original format)")
    print(f"Output file: {output_file}")

if __name__ == "__main__":
    generate_straight_line_trajectory()
