"""
Configuration parameters for the drone swarm simulation.

All distances are in meters, angles in degrees, time in seconds.
These mirror the parameters from flapper_swarm.c but converted to SI units.
"""
from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class FlightParams:
    """Flight parameters matching the C firmware."""
    
    # Target height (not used in 2D sim, but kept for completeness)
    target_height_m: float = 1.0
    
    # Forward speed in m/s
    fwd_speed_mps: float = 0.5
    
    # Outer emergency boundary to beacon (meters) - land if exceeded
    dist0_abort_m: float = 4.2
    
    # Inner boundary (meters) - start turning when exceeded
    inner_bound_m: float = 1.75
    
    # Yaw rate while turning (deg/s)
    turn_yaw_rate_dps: float = 40.0
    
    # Peer distance threshold for avoidance (meters)
    peer_close_m: float = 2.0
    
    # Minimum peer distance before emergency land (meters)
    avoid_min_land_m: float = 0.6
    
    # Speed factor during avoidance (multiplier)
    avoid_speed_factor: float = 1.0
    
    # Yaw rate magnitude for avoidance (deg/s)
    avoid_yaw_rate_dps: float = 70.0
    
    # Confirmation counts (number of consecutive samples to trigger)
    abort_confirm_count: int = 2
    avoid_enter_confirm_count: int = 2
    avoid_exit_confirm_count: int = 4
    
    # Derivative-based recovery parameters
    des_deriv_mps: float = -1.4  # Desired derivative (m/s, negative = toward beacon)
    recover_yaw_rate_dps: float = 50.0  # Yaw rate at 0 derivative
    recover_deadzone_dps: float = 30.0  # Stop rotating within this yaw rate
    
    # Demo duration in seconds
    demo_time_s: float = 60.0


@dataclass
class SimulationParams:
    """Simulation-specific parameters."""
    
    # Time step in seconds
    dt: float = 0.02  # 50 Hz, matching the 20ms loop in firmware
    
    # Derivative buffer size (for linear regression)
    deriv_buffer_size: int = 10
    
    # Derivative sample interval in seconds
    deriv_sample_interval_s: float = 0.02


@dataclass
class NoiseParams:
    """
    Noise parameters for realistic simulation.
    
    Process noise models imperfect velocity tracking due to estimation errors.
    Sensor noise models UWB distance measurement noise.
    """
    
    # === Process Noise (velocity tracking errors) ===
    # These are standard deviations of Gaussian noise added to velocity commands
    
    # Forward velocity noise (m/s) - how much vx deviates from commanded
    process_vx_std: float = 0.02
    
    # Lateral velocity noise (m/s) - drift in y direction
    # This is typically larger due to worse lateral velocity estimation
    process_vy_std: float = 0.05
    
    # Yaw rate noise (deg/s) - how much yaw rate deviates from commanded
    process_yaw_rate_std: float = 2.0
    
    # Lateral velocity bias (m/s) - constant drift in y direction
    # Set to 0 for no bias, or small value like 0.01-0.05 for drift
    process_vy_bias: float = 0.0
    
    # === Sensor Noise (UWB distance measurements) ===
    # UWB noise is modeled as Gaussian with optional bias
    
    # Distance measurement noise standard deviation (meters)
    uwb_distance_std: float = 0.05
    
    # Distance measurement bias (meters) - systematic error
    uwb_distance_bias: float = 0.0

    # === Enable/Disable Flags ===
    enable_process_noise: bool = True
    enable_sensor_noise: bool = True


@dataclass
class VisualizationParams:
    """Pygame visualization parameters."""
    
    # Window size in pixels
    window_size: Tuple[int, int] = (800, 800)
    
    # Scale: pixels per meter
    pixels_per_meter: float = 80.0
    
    # Target FPS
    fps: int = 50
    
    # Colors (RGB)
    background_color: Tuple[int, int, int] = (30, 30, 30)
    beacon_color: Tuple[int, int, int] = (255, 255, 0)
    inner_bound_color: Tuple[int, int, int] = (0, 100, 0)
    outer_bound_color: Tuple[int, int, int] = (100, 0, 0)
    drone1_color: Tuple[int, int, int] = (0, 150, 255)
    drone2_color: Tuple[int, int, int] = (255, 100, 0)
    avoidance_color: Tuple[int, int, int] = (255, 0, 255)
    text_color: Tuple[int, int, int] = (255, 255, 255)
    
    # Drone visual size in pixels
    drone_radius: int = 10
    
    # Show trajectory trail
    show_trail: bool = True
    trail_length: int = 200


@dataclass
class DroneInitialState:
    """Initial state for a drone."""
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    yaw: float = 0.0  # degrees


@dataclass
class Config:
    """Main configuration container."""
    flight: FlightParams = field(default_factory=FlightParams)
    sim: SimulationParams = field(default_factory=SimulationParams)
    viz: VisualizationParams = field(default_factory=VisualizationParams)
    noise: NoiseParams = field(default_factory=NoiseParams)
    
    # Initial states for drones (can be extended for more drones)
    drone1_init: DroneInitialState = field(
        default_factory=lambda: DroneInitialState(x=0.5, y=0.0, yaw=90.0)
    )
    drone2_init: DroneInitialState = field(
        default_factory=lambda: DroneInitialState(x=-0.5, y=0.0, yaw=-90.0)
    )
    
    # Beacon position (center of the arena)
    beacon_pos: Tuple[float, float] = (0.0, 0.0)


# Default configuration instance
default_config = Config()
