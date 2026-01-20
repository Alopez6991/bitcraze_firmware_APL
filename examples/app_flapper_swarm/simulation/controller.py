"""
Flight controller for the drone swarm simulation.

This module implements the control logic from flapper_swarm.c as a proper state machine.
The controller mirrors the C firmware's state machine structure with:
  - STATE_STRAIGHT: Flying straight forward
  - STATE_TURN: Turning at boundary (90° arc)
  - STATE_AVOID: Avoiding peer drone
  - STATE_RECOVER: Recovery mode to return to inner circle

Each state has: on_enter, on_exit, check_transition, execute
"""
import math
from dataclasses import dataclass, field
from typing import Tuple, Optional
from enum import Enum, auto
from collections import deque

from .config import FlightParams, SimulationParams


class FlightState(Enum):
    """Flight state of the drone (matching C enum)."""
    STRAIGHT = 0
    TURN = 1
    AVOID = 2
    RECOVER = 3


@dataclass
class ControlCommand:
    """Velocity command output from the controller."""
    vx_body: float = 0.0  # Forward velocity (m/s)
    vy_body: float = 0.0  # Lateral velocity (m/s)
    yaw_rate: float = 0.0  # Yaw rate (deg/s)


@dataclass
class StateContext:
    """State machine context (shared between state handlers)."""
    # Arc tracking (TURN state)
    arc_active: bool = False
    arc_cooldown: bool = False
    arc_yaw_start: float = 0.0
    target_yaw: float = 0.0
    target_dir: int = 0
    
    # Routine counter
    routines: int = 0
    
    # Current sensor readings (updated each loop)
    d0: float = 0.0  # Distance to beacon (meters)
    d0_deriv: float = 0.0  # Derivative of d0 (m/s)
    peer_dist: float = 0.0  # Distance to peer drone (meters)
    peer_dist_deriv: float = 0.0  # Derivative of peer distance (m/s, negative = closing)


class DerivativeEstimator:
    """
    Estimates derivative of distance using linear regression.
    
    Uses a circular buffer of samples to compute the slope via least squares.
    Matches the C firmware's d0BufferGetDerivative() function.
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
    Controller implementing the flapper swarm algorithm as a state machine.
    
    Each drone has its own controller instance. The controller makes decisions
    based on distance to beacon and distance to peer drone.
    
    State machine structure matches flapper_swarm.c exactly.
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
        
        # Current state
        self.current_state = FlightState.STRAIGHT
        
        # State context (shared data)
        self.ctx = StateContext()
        
        # Confirmation counters
        self.abort_over_count = 0
        self.inner_over_count = 0
        self.inner_under_count = 0
        self.approach_count = 0
        self.depart_count = 0
        
        # Emergency flag
        self.seq_abort = False
        
        # Derivative estimator for beacon distance
        self.d0_deriv = DerivativeEstimator(
            buffer_size=sim_params.deriv_buffer_size,
            sample_interval=sim_params.deriv_sample_interval_s
        )
        
        # Derivative estimator for peer distance (for cooperative avoidance)
        self.peer_dist_deriv = DerivativeEstimator(
            buffer_size=sim_params.deriv_buffer_size,
            sample_interval=sim_params.deriv_sample_interval_s
        )
        
        # Compute recover factor (matching C code: recoverFactor = -(RECOVER_YAWRATE / DES_DERIV))
        self.recover_factor = -(params.recover_yaw_rate_dps / params.des_deriv_mps)
    
    def reset(self) -> None:
        """Reset controller state."""
        self.current_state = FlightState.STRAIGHT
        self.ctx = StateContext()
        self.abort_over_count = 0
        self.inner_over_count = 0
        self.inner_under_count = 0
        self.approach_count = 0
        self.depart_count = 0
        self.seq_abort = False
        self.d0_deriv.reset()
        self.peer_dist_deriv.reset()
        
        # Call onEnter for initial state
        self._on_enter_state(self.current_state)
    
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
        
        # Update derivative buffers
        self.d0_deriv.add_sample(d0, time)
        self.peer_dist_deriv.add_sample(peer_dist, time)
        
        # Update context with current sensor readings
        self.ctx.d0 = d0
        self.ctx.d0_deriv = self.d0_deriv.get_derivative()
        self.ctx.peer_dist = peer_dist
        self.ctx.peer_dist_deriv = self.peer_dist_deriv.get_derivative()
        
        # Check emergency conditions
        should_land = self._check_emergency(d0, peer_dist)
        if should_land:
            return ControlCommand(), True
        
        # Clear arc cooldown once we re-enter the inner circle
        if self.ctx.arc_cooldown and d0 > 0 and d0 <= p.inner_bound_m:
            self.ctx.arc_cooldown = False
        
        # State machine: check transitions
        next_state = self._check_transition(self.current_state, current_yaw)
        
        if next_state != self.current_state:
            self._on_exit_state(self.current_state)
            self.current_state = next_state
            self._on_enter_state(self.current_state, current_yaw)
        
        if self.seq_abort:
            return ControlCommand(), True
        
        # Execute current state
        cmd = self._execute_state(self.current_state)
        
        return cmd, False
    
    # =========================================================================
    # Emergency checks
    # =========================================================================
    
    def _check_emergency(self, d0: float, peer_dist: float) -> bool:
        """
        Check emergency conditions (matching C firmware).
        
        Returns:
            True if should emergency land
        """
        p = self.params
        
        # Check outer boundary (distance0 abort)
        if d0 > p.dist0_abort_m:
            self.abort_over_count += 1
            if self.abort_over_count >= p.abort_confirm_count:
                self.seq_abort = True
                return True
        else:
            self.abort_over_count = 0
        
        # Check peer collision during AVOID state
        if (self.current_state == FlightState.AVOID and 
            peer_dist > 0 and peer_dist <= p.avoid_min_land_m):
            self.seq_abort = True
            return True
        
        return False
    
    # =========================================================================
    # Avoidance confirmation logic (matching C firmware)
    # =========================================================================
    
    def _should_enter_avoid(self) -> bool:
        """Check if peer is too close and we should enter AVOID state."""
        p = self.params
        peer_dist = self.ctx.peer_dist
        
        if peer_dist > 0 and peer_dist <= p.peer_close_m:
            # print(self.ctx.peer_dist_deriv)
            # if (self.ctx.peer_dist_deriv <= -0.3):
            return True
        return False
    
    def _should_exit_avoid(self) -> bool:
        """Check if peer is far enough and we should exit AVOID state."""
        p = self.params
        peer_dist = self.ctx.peer_dist
        
        if peer_dist >= p.peer_close_m:
            return True
        return False
    
    # =========================================================================
    # State machine: on_enter handlers
    # =========================================================================
    
    def _on_enter_state(self, state: FlightState, current_yaw: float = 0.0) -> None:
        """Called when entering a state."""
        if state == FlightState.STRAIGHT:
            self._on_enter_straight()
        elif state == FlightState.TURN:
            self._on_enter_turn(current_yaw)
        elif state == FlightState.AVOID:
            self._on_enter_avoid()
        elif state == FlightState.RECOVER:
            self._on_enter_recover()
    
    def _on_enter_straight(self) -> None:
        """Enter STRAIGHT state."""
        self.inner_over_count = 0
    
    def _on_enter_turn(self, current_yaw: float) -> None:
        """Enter TURN state."""
        self.inner_under_count = 0
        self.ctx.arc_yaw_start = current_yaw
        self.ctx.target_dir = 1 
        self.ctx.target_yaw = self._normalize_angle(current_yaw - self.ctx.target_dir * 90.0)
        self.ctx.arc_active = math.isfinite(current_yaw)
    
    def _on_enter_avoid(self) -> None:
        """Enter AVOID state."""
        self.ctx.target_dir = self._get_rotation_dir()
        pass  # Nothing special in C
    
    def _on_enter_recover(self) -> None:
        """Enter RECOVER state."""
        # If the distance to central beacon is larger than max distance - turning radius, we should keep the direction
        print(f"d0: {self.ctx.d0}, bound: {self.params.dist0_abort_m - 0.8}")
        if (self.ctx.d0 < self.params.dist0_abort_m - 0.8):
            self.ctx.target_dir = 1
    
    # =========================================================================
    # State machine: on_exit handlers
    # =========================================================================
    
    def _on_exit_state(self, state: FlightState) -> None:
        """Called when exiting a state."""
        if state == FlightState.STRAIGHT:
            self._on_exit_straight()
        elif state == FlightState.TURN:
            self._on_exit_turn()
        elif state == FlightState.AVOID:
            self._on_exit_avoid()
        elif state == FlightState.RECOVER:
            self._on_exit_recover()
    
    def _on_exit_straight(self) -> None:
        """Exit STRAIGHT state."""
        pass  # Nothing special in C
    
    def _on_exit_turn(self) -> None:
        """Exit TURN state."""
        self.ctx.arc_active = False
    
    def _on_exit_avoid(self) -> None:
        """Exit AVOID state."""
        self.approach_count = 0
        self.depart_count = 0
    
    def _on_exit_recover(self) -> None:
        """Exit RECOVER state."""
        pass  # Nothing special in C
    
    # =========================================================================
    # State machine: check_transition handlers
    # =========================================================================
    
    def _check_transition(self, state: FlightState, current_yaw: float) -> FlightState:
        """Check for state transitions."""
        if state == FlightState.STRAIGHT:
            return self._check_transition_straight()
        elif state == FlightState.TURN:
            return self._check_transition_turn(current_yaw)
        elif state == FlightState.AVOID:
            return self._check_transition_avoid()
        elif state == FlightState.RECOVER:
            return self._check_transition_recover()
        return state
    
    def _check_transition_straight(self) -> FlightState:
        """Check transitions from STRAIGHT state."""
        p = self.params
        ctx = self.ctx
        
        # STRAIGHT -> AVOID: peer too close
        if self._should_enter_avoid():
            return FlightState.AVOID
        
        # STRAIGHT -> TURN: reached outer bound (only if not in arc cooldown)
        if ctx.d0 >= p.inner_bound_m and not ctx.arc_cooldown:
            self.inner_over_count += 1
            if self.inner_over_count >= p.abort_confirm_count:
                return FlightState.TURN
        elif ctx.d0 < p.inner_bound_m:
            self.inner_over_count = 0
        
        # STRAIGHT -> RECOVER: outside inner bound and moving away from beacon (during cooldown)
        if ctx.d0 >= p.inner_bound_m and ctx.d0_deriv > 0 and ctx.arc_cooldown:
            return FlightState.RECOVER
        
        return FlightState.STRAIGHT
    
    def _check_transition_turn(self, current_yaw: float) -> FlightState:
        """Check transitions from TURN state."""
        p = self.params
        ctx = self.ctx
        
        # TURN -> AVOID: peer too close
        if self._should_enter_avoid():
            return FlightState.AVOID
        
        # TURN -> STRAIGHT: back inside inner bound
        if ctx.d0 <= p.inner_bound_m:
            self.inner_under_count += 1
            if self.inner_under_count >= p.abort_confirm_count:
                ctx.routines += 1
                return FlightState.STRAIGHT
        else:
            self.inner_under_count = 0
        
        # Arc tracking: when 90° reached, decide STRAIGHT or RECOVER
        if ctx.arc_active and math.isfinite(current_yaw):
            yaw_diff = abs(current_yaw - ctx.target_yaw)
            # Handle wrap-around
            if yaw_diff > 180:
                yaw_diff = 360 - yaw_diff
            
            reached = yaw_diff <= 3.0
            if reached:
                ctx.arc_active = False
                ctx.arc_cooldown = True
                if ctx.d0_deriv <= 0:
                    return FlightState.STRAIGHT
                else:
                    return FlightState.RECOVER
        elif ctx.arc_active:
            ctx.arc_active = False  # Yaw unavailable
        
        return FlightState.TURN
    
    def _check_transition_avoid(self) -> FlightState:
        """Check transitions from AVOID state."""
        p = self.params
        ctx = self.ctx
        
        # AVOID -> STRAIGHT or RECOVER: peer far enough
        if self._should_exit_avoid():
            if ctx.d0 < p.inner_bound_m:
                return FlightState.STRAIGHT
            else:
                return FlightState.RECOVER
        
        return FlightState.AVOID
    
    def _check_transition_recover(self) -> FlightState:
        """Check transitions from RECOVER state."""
        p = self.params
        ctx = self.ctx
        
        # RECOVER -> AVOID: peer too close
        if self._should_enter_avoid():
            return FlightState.AVOID
        
        # RECOVER -> STRAIGHT: back inside inner bound
        if ctx.d0 > 0 and ctx.d0 <= p.inner_bound_m:
            return FlightState.STRAIGHT
        
        return FlightState.RECOVER
    
    # =========================================================================
    # State machine: execute handlers
    # =========================================================================
    
    def _execute_state(self, state: FlightState) -> ControlCommand:
        """Execute current state and return command."""
        if state == FlightState.STRAIGHT:
            return self._execute_straight()
        elif state == FlightState.TURN:
            return self._execute_turn()
        elif state == FlightState.AVOID:
            return self._execute_avoid()
        elif state == FlightState.RECOVER:
            return self._execute_recover()
        return ControlCommand()
    
    def _execute_straight(self) -> ControlCommand:
        """Execute STRAIGHT state."""
        return ControlCommand(
            vx_body=self.params.fwd_speed_mps,
            vy_body=0.0,
            yaw_rate=0.0
        )
    
    def _execute_turn(self) -> ControlCommand:
        """Execute TURN state."""
        return ControlCommand(
            vx_body=self.params.fwd_speed_mps,
            vy_body=0.0,
            yaw_rate=self.params.turn_yaw_rate_dps
        )
    
    def _execute_avoid(self) -> ControlCommand:
        """Execute AVOID state."""
        return ControlCommand(
            vx_body=self.params.fwd_speed_mps * self.params.avoid_speed_factor,
            vy_body=0.0,
            yaw_rate=self._get_avoid_yaw_rate()
        )
    
    def _execute_recover(self) -> ControlCommand:
        """Execute RECOVER state."""
        p = self.params
        ctx = self.ctx
        
        # Yaw rate based on derivative error (matching C code)
        # yawCommand = recoverFactor * fabsf(ctx.d0Deriv - DES_DERIV)
        yaw_cmd = self.recover_factor * abs(ctx.d0_deriv - p.des_deriv_mps)
        yaw_cmd *= self.ctx.target_dir # keep the last direction to prevent exiting

        # Apply deadzone
        if abs(yaw_cmd) <= p.recover_deadzone_dps:
            yaw_cmd = 0.0
        
        return ControlCommand(
            vx_body=p.fwd_speed_mps,
            vy_body=0.0,
            yaw_rate=yaw_cmd
        )
    
    # =========================================================================
    # Helper methods
    # =========================================================================
    def _get_rotation_dir(self) -> int:
        """Get the rotation direction based on drone ID."""
        if self.drone_id == 1:
            return 1
        else:
            return -1

    def _get_avoid_yaw_rate(self) -> float:
        """Get avoidance yaw rate based on drone ID.""" 
        # mod_factor = 0
        # if (self.ctx.peer_dist_deriv > 0):
        #     mod_factor = abs(self.ctx.peer_dist_deriv / 2) * self.params.avoid_yaw_rate_dps
        # if self.drone_id == 1:
        #     return self.params.avoid_yaw_rate_dps - mod_factor # CW (positive)
        # else:
        #     return -self.params.avoid_yaw_rate_dps + mod_factor # CCW (negative)
        if self.drone_id == 1:
            return self.params.avoid_yaw_rate_dps # CW (positive)
        else:
            return -self.params.avoid_yaw_rate_dps # CCW (negative)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle
    
    # =========================================================================
    # Properties for external access
    # =========================================================================
    
    @property
    def mode(self) -> FlightState:
        """Get current flight state."""
        return self.current_state
    
    @property
    def is_avoiding(self) -> bool:
        """Check if currently in AVOID state."""
        return self.current_state == FlightState.AVOID


# Keep FlightMode as alias for backward compatibility with simulator
FlightMode = FlightState
