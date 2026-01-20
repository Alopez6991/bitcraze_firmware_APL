#!/usr/bin/env python3
"""
Run the Flapper Swarm Simulation.

This is the main entry point for the 2D simulation.
Run with: python run_sim.py

Controls:
  SPACE - Pause/Resume
  R     - Reset simulation
  ESC   - Quit
"""
import sys
import os

# Add parent directory to path for imports when running as module
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
sys.path.insert(0, parent_dir)

from simulation import run_simulation, Config, FlightParams, VisualizationParams, DroneInitialState


def main():
    """Run the simulation with custom configuration."""
    
    # You can customize the configuration here
    config = Config(
        flight=FlightParams(
            # Forward speed in m/s
            fwd_speed_mps=0.5,
            
            # Outer emergency boundary (meters) - land if exceeded
            dist0_abort_m=3,
            
            # Inner boundary (meters) - start turning when exceeded
            inner_bound_m=1.30,
            
            # Yaw rate while turning (deg/s)
            turn_yaw_rate_dps=40.0,
            
            # Peer distance threshold for avoidance (meters)
            peer_close_m=2.0,
            
            # Minimum peer distance before emergency land (meters)
            avoid_min_land_m=0.6,
            
            # Yaw rate for avoidance maneuver (deg/s)
            avoid_yaw_rate_dps=70.0,
            
            # Demo duration (seconds)
            demo_time_s=60.0,
        ),
        viz=VisualizationParams(
            # Window size
            window_size=(900, 900),
            
            # Scale: pixels per meter
            pixels_per_meter=90.0,
            
            # Show trajectory trails
            show_trail=True,
            trail_length=300,
        ),
        # Initial positions and headings for drones
        drone1_init=DroneInitialState(x=0.8, y=0.0, yaw=270.0),
        drone2_init=DroneInitialState(x=-0.8, y=0.0, yaw=-90.0),
    )
    
    print("=" * 50)
    print("Flapper Swarm Simulation")
    print("=" * 50)
    print()
    print("Controls:")
    print("  SPACE - Pause/Resume")
    print("  R     - Reset simulation")
    print("  ESC   - Quit")
    print()
    print("Configuration:")
    print(f"  Inner boundary: {config.flight.inner_bound_m}m")
    print(f"  Outer boundary: {config.flight.dist0_abort_m}m")
    print(f"  Avoidance distance: {config.flight.peer_close_m}m")
    print(f"  Forward speed: {config.flight.fwd_speed_mps}m/s")
    print()
    
    run_simulation(config)


if __name__ == "__main__":
    main()
