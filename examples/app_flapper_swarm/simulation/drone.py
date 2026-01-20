"""
Drone physics model for 2D simulation.

This module contains the Drone class which handles the physical state
and kinematics of a drone. The physics model is kept simple and modular
so it can be extended with dynamics later.
"""
import math
from dataclasses import dataclass, field
from typing import Tuple, List, Protocol
from enum import Enum, auto


class DroneState(Enum):
    """Flight state of the drone."""
    IDLE = auto()
    FLYING = auto()
    LANDED = auto()


@dataclass
class DronePhysicsState:
    """Physical state of a drone in 2D."""
    x: float = 0.0  # Position X (meters)
    y: float = 0.0  # Position Y (meters)
    yaw: float = 0.0  # Heading (degrees, 0 = +X axis, CCW positive)
    vx: float = 0.0  # Velocity X (m/s) - world frame
    vy: float = 0.0  # Velocity Y (m/s) - world frame
    yaw_rate: float = 0.0  # Yaw rate (deg/s)


class PhysicsModel(Protocol):
    """Protocol for physics models (allows swapping different dynamics)."""
    
    def update(
        self,
        state: DronePhysicsState,
        cmd_vx_body: float,
        cmd_vy_body: float,
        cmd_yaw_rate: float,
        dt: float
    ) -> DronePhysicsState:
        """Update physics state based on commands."""
        ...


class KinematicPhysics:
    """
    Simple kinematic physics model.
    
    Commands translate directly to state without any dynamics/lag.
    This can be replaced with a more complex model later.
    """
    
    def update(
        self,
        state: DronePhysicsState,
        cmd_vx_body: float,
        cmd_vy_body: float,
        cmd_yaw_rate: float,
        dt: float
    ) -> DronePhysicsState:
        """
        Update physics state based on velocity commands.
        
        Args:
            state: Current physics state
            cmd_vx_body: Commanded forward velocity (m/s) in body frame
            cmd_vy_body: Commanded lateral velocity (m/s) in body frame
            cmd_yaw_rate: Commanded yaw rate (deg/s)
            dt: Time step (seconds)
            
        Returns:
            New physics state
        """
        # Convert yaw to radians for trig
        yaw_rad = math.radians(state.yaw)
        
        # Transform body velocities to world frame
        vx_world = cmd_vx_body * math.cos(yaw_rad) - cmd_vy_body * math.sin(yaw_rad)
        vy_world = cmd_vx_body * math.sin(yaw_rad) + cmd_vy_body * math.cos(yaw_rad)
        
        # Update state
        new_state = DronePhysicsState(
            x=state.x + vx_world * dt,
            y=state.y + vy_world * dt,
            yaw=self._normalize_angle(state.yaw + cmd_yaw_rate * dt),
            vx=vx_world,
            vy=vy_world,
            yaw_rate=cmd_yaw_rate
        )
        
        return new_state
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle


class Drone:
    """
    Represents a drone in the simulation.
    
    Handles physical state, position history for trails, and distance calculations.
    The physics model is pluggable for easy extension.
    """
    
    def __init__(
        self,
        drone_id: int,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_yaw: float = 0.0,
        physics_model: PhysicsModel = None,
        trail_length: int = 200
    ):
        """
        Initialize a drone.
        
        Args:
            drone_id: Unique identifier (1 or 2 for the two-drone scenario)
            initial_x: Initial X position (meters)
            initial_y: Initial Y position (meters)
            initial_yaw: Initial heading (degrees)
            physics_model: Physics model to use (default: KinematicPhysics)
            trail_length: Number of positions to store for trail visualization
        """
        self.drone_id = drone_id
        self.physics = physics_model or KinematicPhysics()
        self.trail_length = trail_length
        
        # Initialize state
        self.state = DronePhysicsState(
            x=initial_x,
            y=initial_y,
            yaw=initial_yaw
        )
        self.flight_state = DroneState.IDLE
        
        # Position history for trail
        self.trail: List[Tuple[float, float]] = []
        
    def update(
        self,
        cmd_vx_body: float,
        cmd_vy_body: float,
        cmd_yaw_rate: float,
        dt: float
    ) -> None:
        """
        Update drone state based on velocity commands.
        
        Args:
            cmd_vx_body: Commanded forward velocity (m/s) in body frame
            cmd_vy_body: Commanded lateral velocity (m/s) in body frame
            cmd_yaw_rate: Commanded yaw rate (deg/s)
            dt: Time step (seconds)
        """
        if self.flight_state != DroneState.FLYING:
            return
            
        # Store position for trail
        self.trail.append((self.state.x, self.state.y))
        if len(self.trail) > self.trail_length:
            self.trail.pop(0)
        
        # Update physics
        self.state = self.physics.update(
            self.state,
            cmd_vx_body,
            cmd_vy_body,
            cmd_yaw_rate,
            dt
        )
    
    def takeoff(self) -> None:
        """Start flying."""
        self.flight_state = DroneState.FLYING
        self.trail.clear()
    
    def land(self) -> None:
        """Land the drone."""
        self.flight_state = DroneState.LANDED
        self.state.vx = 0.0
        self.state.vy = 0.0
        self.state.yaw_rate = 0.0
    
    def distance_to(self, x: float, y: float) -> float:
        """Calculate distance to a point."""
        dx = self.state.x - x
        dy = self.state.y - y
        return math.sqrt(dx * dx + dy * dy)
    
    def distance_to_drone(self, other: 'Drone') -> float:
        """Calculate distance to another drone."""
        return self.distance_to(other.state.x, other.state.y)
    
    @property
    def position(self) -> Tuple[float, float]:
        """Get current position."""
        return (self.state.x, self.state.y)
    
    @property
    def yaw(self) -> float:
        """Get current heading in degrees."""
        return self.state.yaw
    
    @property
    def is_flying(self) -> bool:
        """Check if drone is currently flying."""
        return self.flight_state == DroneState.FLYING
    
    def reset(self, x: float, y: float, yaw: float) -> None:
        """Reset drone to initial state."""
        self.state = DronePhysicsState(x=x, y=y, yaw=yaw)
        self.flight_state = DroneState.IDLE
        self.trail.clear()
