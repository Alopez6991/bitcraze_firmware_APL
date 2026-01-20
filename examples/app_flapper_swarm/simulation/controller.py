"""
Flight controller for the drone swarm simulation.

This module implements the control logic from flapper_swarm.c.
The controller is modular and can be swapped for different algorithms.
"""
import math
from dataclasses import dataclass, field
from typing import Tuple, List, Optional, Protocol
from enum import Enum, auto
from collections import deque

from .config import FlightParams, SimulationParams


class FlightMode(Enum):
    """Flight mode of the drone."""
    STRAIGHT = auto()  # Flying straight forward
    TURN = auto()       # Turning at boundary
    RECOVER = auto()    # Recovery mode after turn


@dataclass
class ControlCommand:
    """Velocity command output from the controller."""
    vx_body: float = 0.0  # Forward velocity (m/s)
    vy_body: float = 0.0  # Lateral velocity (m/s)
    yaw_rate: float = 0.0  # Yaw rate (deg/s)


@dataclass
class ControllerState:
    """Internal state of the controller."""
    mode: FlightMode = FlightMode.STRAIGHT
    
    # Confirmation counters
    abort_over_count: int = 0
    inner_over_count: int = 0
    inner_under_count: int = 0
    
    # Avoidance state
    avoid_active: bool = False
    approach_count: int = 0
    depart_count: int = 0
    avoid_was_active: bool = False
    
    # Arc tracking for turns
    arc_active: bool = False
    arc_cooldown: bool = False
    arc_yaw_start: float = 0.0
    target_yaw: float = 0.0
    
    # Emergency flag
    seq_abort: bool = False
    
    # Derivative buffer for d0
    d0_buffer: deque = field(default_factory=lambda: deque(maxlen=10))
    last_sample_time: float = 0.0


class DerivativeEstimator:
    """
    Estimates derivative of distance using linear regression.
    
    Uses a circular buffer of samples to compute the slope via least squares.
    """
    
    def __init__(self, buffer_size: int = 10, sample_interval: float = 0.02):
        """
        Initialize the derivative estimator.
        
        Args:
            buffer_size: Number of samples to keep
            sample_interval: Minimum time between samples (seconds)
        """
        self.buffer_size = buffer_size
        self.sample_interval = sample_interval
        self.samples: deque = deque(maxlen=buffer_size)
        self.last_sample_time: float = -float('inf')
    
    def add_sample(self, value: float, time: float) -> None:
        """
        Add a sample if enough time has passed.
        
        Args:
            value: Distance value (meters)
            time: Current time (seconds)
        """
        if time - self.last_sample_time >= self.sample_interval:
            self.samples.append(value)
            self.last_sample_time = time
    
    def get_derivative(self) -> float:
        """
        Compute derivative using linear regression.
        
        Returns:
            Derivative in m/s (positive = moving away, negative = moving toward)
        """
        n = len(self.samples)
        if n < 2:
            return 0.0
        
        # Compute sums for least squares
        sum_t = 0.0
        sum_y = 0.0
        sum_ty = 0.0
        sum_t2 = 0.0
        
        for i, y in enumerate(self.samples):
            t = float(i)
            sum_t += t
            sum_y += y
            sum_ty += t * y
            sum_t2 += t * t
        
        denom = n * sum_t2 - sum_t * sum_t
        if abs(denom) < 1e-6:
            return 0.0
        
        slope_per_interval = (n * sum_ty - sum_t * sum_y) / denom
        
        # Convert to per-second rate
        return slope_per_interval / self.sample_interval
    
    def reset(self) -> None:
        """Clear the buffer."""
        self.samples.clear()
        self.last_sample_time = -float('inf')


class SwarmController:
    """
    Controller implementing the flapper swarm algorithm.
    
    Each drone has its own controller instance. The controller makes decisions
    based on distance to beacon and distance to peer drone.
    """
    
    def __init__(
        self,
        drone_id: int,
        params: FlightParams,
        sim_params: SimulationParams
    ):
        """
        Initialize the controller.
        
        Args:
            drone_id: Drone identifier (1 or 2)
            params: Flight parameters
            sim_params: Simulation parameters
        """
        self.drone_id = drone_id
        self.params = params
        self.sim_params = sim_params
        
        # Internal state
        self.state = ControllerState()
        
        # Derivative estimator for beacon distance
        self.d0_deriv = DerivativeEstimator(
            buffer_size=sim_params.deriv_buffer_size,
            sample_interval=sim_params.deriv_sample_interval_s
        )
        
        # Compute recover factor (matching C code)
        self.recover_factor = -(params.recover_yaw_rate_dps / params.des_deriv_mps)
    
    def reset(self) -> None:
        """Reset controller state."""
        self.state = ControllerState()
        self.d0_deriv.reset()
    
    def update(
        self,
        d0: float,  # Distance to beacon (meters)
        peer_dist: float,  # Distance to peer drone (meters)
        current_yaw: float,  # Current heading (degrees)
        time: float  # Current simulation time (seconds)
    ) -> Tuple[ControlCommand, bool]:
        """
        Update controller and get command.
        
        Args:
            d0: Distance to beacon (meters)
            peer_dist: Distance to peer drone (meters)
            current_yaw: Current heading (degrees)
            time: Current simulation time (seconds)
            
        Returns:
            Tuple of (ControlCommand, should_land)
        """
        p = self.params
        
        # Add to derivative buffer
        self.d0_deriv.add_sample(d0, time)
        d0_deriv = self.d0_deriv.get_derivative()
        
        # Check emergency conditions
        should_land = self._check_emergency(d0, peer_dist)
        if should_land:
            return ControlCommand(), True
        
        # Clear arc cooldown once we re-enter the inner circle
        if self.state.arc_cooldown and d0 <= p.inner_bound_m:
            self.state.arc_cooldown = False
        
        # Mode transitions
        self._update_mode_transitions(d0, current_yaw, d0_deriv)
        
        # Update avoidance mode
        self.state.avoid_was_active = self.state.avoid_active
        avoid = self._update_avoidance(peer_dist)
        
        # After exiting avoidance, if outside inner bound, go to recover
        if self.state.avoid_was_active and not avoid and d0 >= p.inner_bound_m:
            self.state.mode = FlightMode.RECOVER
        
        # Generate command based on mode
        cmd = self._generate_command(avoid, d0, d0_deriv)
        
        return cmd, False
    
    def _check_emergency(self, d0: float, peer_dist: float) -> bool:
        """
        Check emergency conditions.
        
        Returns:
            True if should emergency land
        """
        p = self.params
        
        # Check outer boundary
        if d0 > p.dist0_abort_m:
            self.state.abort_over_count += 1
            if self.state.abort_over_count >= p.abort_confirm_count:
                self.state.seq_abort = True
                return True
        else:
            self.state.abort_over_count = 0
        
        # Check peer collision during avoidance
        if self.state.avoid_active and peer_dist > 0 and peer_dist <= p.avoid_min_land_m:
            self.state.seq_abort = True
            return True
        
        return False
    
    def _update_mode_transitions(
        self,
        d0: float,
        current_yaw: float,
        d0_deriv: float
    ) -> None:
        """Update flight mode based on distance to beacon."""
        p = self.params
        
        if self.state.mode == FlightMode.STRAIGHT:
            # Check if we should start turning
            if d0 >= p.inner_bound_m:
                if not self.state.arc_cooldown:
                    self.state.inner_over_count += 1
                    if self.state.inner_over_count >= p.abort_confirm_count:
                        self.state.mode = FlightMode.TURN
                        self.state.inner_over_count = 0
                        self.state.arc_yaw_start = current_yaw
                        self.state.target_yaw = self._normalize_angle(current_yaw - 90.0)
                        self.state.arc_active = True
            else:
                self.state.inner_over_count = 0
                
        elif self.state.mode == FlightMode.TURN:
            # Check if we should exit turn
            if d0 <= p.inner_bound_m:
                self.state.inner_under_count += 1
                if self.state.inner_under_count >= p.abort_confirm_count:
                    self.state.mode = FlightMode.STRAIGHT
                    self.state.inner_under_count = 0
            else:
                self.state.inner_under_count = 0
            
            # Arc tracking: check if we've reached target yaw
            if self.state.arc_active:
                yaw_diff = abs(current_yaw - self.state.target_yaw)
                # Handle wrap-around
                if yaw_diff > 180:
                    yaw_diff = 360 - yaw_diff
                
                if yaw_diff <= 3.0:
                    self.state.arc_active = False
                    self.state.arc_cooldown = True
                    self.state.inner_under_count = 0
                    
                    # Check if we need recovery
                    if d0_deriv > 0:
                        self.state.mode = FlightMode.RECOVER
                    else:
                        self.state.mode = FlightMode.STRAIGHT
        
        elif self.state.mode == FlightMode.RECOVER:
            # Exit recovery when back inside inner bound
            if d0 <= p.inner_bound_m:
                self.state.mode = FlightMode.STRAIGHT
    
    def _update_avoidance(self, peer_dist: float) -> bool:
        """
        Update avoidance mode based on peer distance.
        
        Returns:
            True if avoidance is active
        """
        p = self.params
        
        if not self.state.avoid_active:
            # Check if we should enter avoidance
            if peer_dist > 0 and peer_dist <= p.peer_close_m:
                self.state.approach_count += 1
                if self.state.approach_count >= p.avoid_enter_confirm_count:
                    self.state.avoid_active = True
                    self.state.approach_count = 0
                    self.state.depart_count = 0
            else:
                self.state.approach_count = 0
        else:
            # Check if we should exit avoidance
            if peer_dist >= p.peer_close_m:
                self.state.depart_count += 1
                if self.state.depart_count >= p.avoid_exit_confirm_count:
                    self.state.avoid_active = False
                    self.state.depart_count = 0
                    self.state.approach_count = 0
            else:
                self.state.depart_count = 0
        
        return self.state.avoid_active
    
    def _generate_command(
        self,
        avoid: bool,
        d0: float,
        d0_deriv: float
    ) -> ControlCommand:
        """Generate velocity command based on current mode."""
        p = self.params
        
        if avoid:
            # Avoidance: fly forward with evasive yaw
            yaw_rate = self._get_avoid_yaw_rate()
            return ControlCommand(
                vx_body=p.fwd_speed_mps * p.avoid_speed_factor,
                vy_body=0.0,
                yaw_rate=yaw_rate
            )
        
        elif self.state.mode == FlightMode.STRAIGHT:
            return ControlCommand(
                vx_body=p.fwd_speed_mps,
                vy_body=0.0,
                yaw_rate=0.0
            )
        
        elif self.state.mode == FlightMode.RECOVER:
            # Yaw rate based on derivative error
            yaw_cmd = self.recover_factor * abs(d0_deriv - p.des_deriv_mps)
            if yaw_cmd <= p.recover_deadzone_dps:
                yaw_cmd = 0.0
            return ControlCommand(
                vx_body=p.fwd_speed_mps,
                vy_body=0.0,
                yaw_rate=yaw_cmd
            )
        
        else:  # TURN mode
            return ControlCommand(
                vx_body=p.fwd_speed_mps,
                vy_body=0.0,
                yaw_rate=p.turn_yaw_rate_dps
            )
    
    def _get_avoid_yaw_rate(self) -> float:
        """Get avoidance yaw rate based on drone ID."""
        if self.drone_id == 1:
            return self.params.avoid_yaw_rate_dps  # CW (positive)
        else:
            return -self.params.avoid_yaw_rate_dps  # CCW (negative)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle
    
    @property
    def mode(self) -> FlightMode:
        """Get current flight mode."""
        return self.state.mode
    
    @property
    def is_avoiding(self) -> bool:
        """Check if avoidance is active."""
        return self.state.avoid_active
