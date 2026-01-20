"""
Flapper Swarm Simulation Package.

A 2D simulation for testing and tuning the drone swarm algorithm.
"""
from .config import (
    Config,
    FlightParams,
    SimulationParams,
    VisualizationParams,
    NoiseParams,
    DroneInitialState,
    default_config
)
from .drone import (
    Drone,
    DroneState,
    DronePhysicsState,
    KinematicPhysics,
    NoisyKinematicPhysics,
    UWBSensor
)
from .controller import SwarmController, FlightMode, ControlCommand
from .simulator import Simulator, run_simulation

__all__ = [
    'Config',
    'FlightParams',
    'SimulationParams',
    'VisualizationParams',
    'NoiseParams',
    'DroneInitialState',
    'default_config',
    'Drone',
    'DroneState',
    'DronePhysicsState',
    'KinematicPhysics',
    'NoisyKinematicPhysics',
    'UWBSensor',
    'SwarmController',
    'FlightMode',
    'ControlCommand',
    'Simulator',
    'run_simulation',
]
