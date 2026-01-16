# Flapper Swarm App for Crazyflie/Flapper

This folder contains a unified swarm application that supports multiple drones with a single codebase.

## Overview

The app supports different drone roles selected at runtime via the persistent parameter `swarm.droneId`:

| droneId | Role | Trigger | Avoidance Distance | Avoidance Yaw |
|---------|------|---------|-------------------|---------------|
| 1 | Primary drone | RC remote (`cppm.aux0`, active low) | `distance2` (to drone 2) | CW (+70°/s) |
| 2 | Secondary drone | UWB (`ranging.aux1`, active high) | `distance1` (to drone 1) | CCW (-70°/s) |

## Common Behavior (All Drones)

- Monitor `distance0` (beacon) for outer boundary emergency land
- Flight pattern: STRAIGHT → TURN → RECOVER
- Kill switch via `ranging.aux2` (drone 2+ only)

## Configuration

Set the drone ID from the Python client before flight:

```python
import cflib.crtp
from cflib.crazyflie import Crazyflie

cf = Crazyflie()
cf.open_link('radio://0/80/2M')

# Set drone ID (persisted across reboots)
cf.param.set_value('swarm.droneId', 1)  # or 2
```

The parameter is persistent, so it survives power cycles after being set once.

## Building

```bash
cd examples/app_flapper_swarm
make clean && make
```

## Flashing

```bash
cfloader flash build/flapper.bin stm32-fw -w radio://0/80/2M
```

See App layer API guide and build instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)
