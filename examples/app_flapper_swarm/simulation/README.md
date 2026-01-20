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
from simulation import Config, FlightParams, run_simulation

config = Config(
    flight=FlightParams(
        fwd_speed_mps=0.5,
        inner_bound_m=1.75,
        dist0_abort_m=4.2,
        peer_close_m=2.0,
        avoid_yaw_rate_dps=70.0,
    )
)

run_simulation(config)
```

## Architecture

The simulation is modular for easy extension:

- **`config.py`** - All tunable hyperparameters
- **`drone.py`** - Drone physics (2D kinematic model, swappable)
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
