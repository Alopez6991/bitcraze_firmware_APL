# Flapper Swarm Simulation

A 2D Python simulation for testing and tuning the drone swarm algorithm used in `flapper_swarm.c`.

## Installation

```bash
pip install pygame
```

## Usage

```bash
cd examples/app_flapper_swarm/simulation
python run_sim.py
```

## Controls

| Key   | Action          |
|-------|-----------------|
| SPACE | Pause/Resume    |
| R     | Reset simulation|
| ESC   | Quit            |

## Configuration

Edit parameters in `run_sim.py` or create your own configuration:

```python
from simulation import Config, FlightParams, NoiseParams, run_simulation

config = Config(
    flight=FlightParams(
        fwd_speed_mps=0.5,
        inner_bound_m=1.75,
        dist0_abort_m=4.2,
        peer_close_m=2.0,
        avoid_yaw_rate_dps=70.0,
    ),
    noise=NoiseParams(
        enable_process_noise=True,
        enable_sensor_noise=True,
        process_vy_std=0.05,  # Lateral drift
        uwb_distance_std=0.05,  # UWB noise
    ),
)

run_simulation(config)
```

## Noise Models

The simulation supports realistic noise modeling:

### Process Noise (Velocity Tracking Errors)

Models imperfect velocity control due to state estimation errors:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_process_noise` | `True` | Enable/disable process noise |
| `process_vx_std` | `0.02` m/s | Forward velocity noise std dev |
| `process_vy_std` | `0.05` m/s | Lateral velocity noise std dev (typically larger) |
| `process_yaw_rate_std` | `2.0` deg/s | Yaw rate noise std dev |
| `process_vy_bias` | `0.0` m/s | Constant lateral drift |

### Sensor Noise (UWB Distance Measurements)

Models UWB ranging measurement noise:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_sensor_noise` | `True` | Enable/disable sensor noise |
| `uwb_distance_std` | `0.05` m | Distance measurement noise std dev |
| `uwb_distance_bias` | `0.0` m | Systematic measurement bias |
| `uwb_outlier_prob` | `0.01` | Probability of outlier (0.0-1.0) |
| `uwb_outlier_magnitude` | `0.5` m | Outlier magnitude range |

## Architecture

The simulation is modular for easy extension:

- **`config.py`** - All tunable hyperparameters (flight, noise, visualization)
- **`drone.py`** - Drone physics (`KinematicPhysics`, `NoisyKinematicPhysics`, `UWBSensor`)
- **`controller.py`** - Flight control algorithm (swappable)
- **`simulator.py`** - Main loop and Pygame visualization
- **`run_sim.py`** - Entry point

### Swapping the Physics Model

The drone uses a `PhysicsModel` protocol. To add dynamics:

```python
from simulation.drone import Drone, DronePhysicsState

class MyDynamicPhysics:
    def update(self, state, cmd_vx, cmd_vy, cmd_yaw_rate, dt):
        # Add your dynamics here
        ...
        return new_state

drone = Drone(drone_id=1, physics_model=MyDynamicPhysics())
```

### Swapping the Controller

Create a new controller class with the same interface as `SwarmController`.

## Visualization

- **Yellow dot**: Center beacon
- **Green circle**: Inner boundary (turn trigger)
- **Red circle**: Outer boundary (emergency land)
- **Blue drone**: Drone 1 (yaws CW during avoidance)
- **Orange drone**: Drone 2 (yaws CCW during avoidance)
- **Magenta**: Avoidance mode active
- **Trails**: Recent trajectory history
