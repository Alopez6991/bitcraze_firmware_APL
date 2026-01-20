"""
Main simulator with Pygame visualization.

This module ties together the drone physics and controller,
and provides a visual simulation using Pygame.
"""
import math
import sys
from typing import List, Tuple, Optional

try:
    import pygame
except ImportError:
    print("Pygame is required. Install with: pip install pygame")
    sys.exit(1)

from .config import Config, default_config
from .drone import Drone, DroneState
from .controller import SwarmController, FlightMode


class Simulator:
    """
    Main simulation class.
    
    Handles the simulation loop, visualization, and coordination
    between drones and their controllers.
    """
    
    def __init__(self, config: Config = None):
        """
        Initialize the simulator.
        
        Args:
            config: Configuration object (uses default if not provided)
        """
        self.config = config or default_config
        self.running = False
        self.paused = False
        self.time = 0.0
        
        # Initialize pygame
        pygame.init()
        pygame.display.set_caption("Flapper Swarm Simulation")
        self.screen = pygame.display.set_mode(self.config.viz.window_size)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        
        # Create drones
        self.drones: List[Drone] = []
        self.controllers: List[SwarmController] = []
        self._init_drones()
        
        # Statistics
        self.min_peer_distance = float('inf')
        self.avoidance_events = 0
    
    def _init_drones(self) -> None:
        """Initialize drones and controllers."""
        cfg = self.config
        
        # Drone 1
        drone1 = Drone(
            drone_id=1,
            initial_x=cfg.drone1_init.x,
            initial_y=cfg.drone1_init.y,
            initial_yaw=cfg.drone1_init.yaw,
            trail_length=cfg.viz.trail_length
        )
        controller1 = SwarmController(
            drone_id=1,
            params=cfg.flight,
            sim_params=cfg.sim
        )
        
        # Drone 2
        drone2 = Drone(
            drone_id=2,
            initial_x=cfg.drone2_init.x,
            initial_y=cfg.drone2_init.y,
            initial_yaw=cfg.drone2_init.yaw,
            trail_length=cfg.viz.trail_length
        )
        controller2 = SwarmController(
            drone_id=2,
            params=cfg.flight,
            sim_params=cfg.sim
        )
        
        self.drones = [drone1, drone2]
        self.controllers = [controller1, controller2]
    
    def reset(self) -> None:
        """Reset simulation to initial state."""
        cfg = self.config
        self.time = 0.0
        self.min_peer_distance = float('inf')
        self.avoidance_events = 0
        
        # Reset drones
        self.drones[0].reset(
            cfg.drone1_init.x,
            cfg.drone1_init.y,
            cfg.drone1_init.yaw
        )
        self.drones[1].reset(
            cfg.drone2_init.x,
            cfg.drone2_init.y,
            cfg.drone2_init.yaw
        )
        
        # Reset controllers
        for ctrl in self.controllers:
            ctrl.reset()
    
    def run(self) -> None:
        """Run the simulation loop."""
        self.running = True
        self.reset()
        
        # Start drones
        for drone in self.drones:
            drone.takeoff()
        
        while self.running:
            self._handle_events()
            
            if not self.paused:
                self._update()
            
            self._render()
            self.clock.tick(self.config.viz.fps)
        
        pygame.quit()
    
    def _handle_events(self) -> None:
        """Handle pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self.reset()
                    for drone in self.drones:
                        drone.takeoff()
    
    def _update(self) -> None:
        """Update simulation state."""
        dt = self.config.sim.dt
        beacon = self.config.beacon_pos
        
        # Check if demo time exceeded
        if self.time >= self.config.flight.demo_time_s:
            for drone in self.drones:
                if drone.is_flying:
                    drone.land()
            return
        
        # Track avoidance state changes
        prev_avoiding = [ctrl.is_avoiding for ctrl in self.controllers]
        
        # Update each drone
        for i, (drone, ctrl) in enumerate(zip(self.drones, self.controllers)):
            if not drone.is_flying:
                continue
            
            # Get other drone for peer distance
            other_drone = self.drones[1 - i]
            
            # Calculate distances
            d0 = drone.distance_to(*beacon)
            peer_dist = drone.distance_to_drone(other_drone)
            
            # Track minimum peer distance
            self.min_peer_distance = min(self.min_peer_distance, peer_dist)
            
            # Get control command
            cmd, should_land = ctrl.update(
                d0=d0,
                peer_dist=peer_dist,
                current_yaw=drone.yaw,
                time=self.time
            )
            
            if should_land:
                drone.land()
                continue
            
            # Apply command to drone
            drone.update(cmd.vx_body, cmd.vy_body, cmd.yaw_rate, dt)
        
        # Count avoidance events
        for i, ctrl in enumerate(self.controllers):
            if ctrl.is_avoiding and not prev_avoiding[i]:
                self.avoidance_events += 1
        
        self.time += dt
    
    def _world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates."""
        viz = self.config.viz
        cx, cy = viz.window_size[0] // 2, viz.window_size[1] // 2
        
        # Note: Y is inverted (screen Y increases downward)
        sx = int(cx + x * viz.pixels_per_meter)
        sy = int(cy - y * viz.pixels_per_meter)
        
        return (sx, sy)
    
    def _render(self) -> None:
        """Render the simulation."""
        viz = self.config.viz
        flight = self.config.flight
        
        # Clear screen
        self.screen.fill(viz.background_color)
        
        # Draw boundaries
        center = self._world_to_screen(*self.config.beacon_pos)
        
        # Outer abort boundary
        outer_radius = int(flight.dist0_abort_m * viz.pixels_per_meter)
        pygame.draw.circle(self.screen, viz.outer_bound_color, center, outer_radius, 2)
        
        # Inner turn boundary
        inner_radius = int(flight.inner_bound_m * viz.pixels_per_meter)
        pygame.draw.circle(self.screen, viz.inner_bound_color, center, inner_radius, 2)
        
        # Peer avoidance radius (for reference)
        avoid_radius = int(flight.peer_close_m * viz.pixels_per_meter)
        # Draw around each drone
        for drone in self.drones:
            if drone.is_flying:
                pos = self._world_to_screen(*drone.position)
                pygame.draw.circle(self.screen, (50, 50, 50), pos, avoid_radius, 1)
        
        # Draw beacon
        pygame.draw.circle(self.screen, viz.beacon_color, center, 8)
        
        # Draw drones
        drone_colors = [viz.drone1_color, viz.drone2_color]
        
        for i, (drone, ctrl) in enumerate(zip(self.drones, self.controllers)):
            color = drone_colors[i]
            
            # Use avoidance color if avoiding
            if ctrl.is_avoiding:
                color = viz.avoidance_color
            
            # Draw trail
            if viz.show_trail and len(drone.trail) > 1:
                trail_points = [self._world_to_screen(x, y) for x, y in drone.trail]
                if len(trail_points) >= 2:
                    pygame.draw.lines(self.screen, color, False, trail_points, 1)
            
            # Draw drone
            pos = self._world_to_screen(*drone.position)
            pygame.draw.circle(self.screen, color, pos, viz.drone_radius)
            
            # Draw heading indicator
            yaw_rad = math.radians(drone.yaw)
            head_len = viz.drone_radius * 2
            head_x = pos[0] + int(head_len * math.cos(yaw_rad))
            head_y = pos[1] - int(head_len * math.sin(yaw_rad))
            pygame.draw.line(self.screen, color, pos, (head_x, head_y), 2)
        
        # Draw HUD
        self._render_hud()
        
        pygame.display.flip()
    
    def _render_hud(self) -> None:
        """Render heads-up display with status info."""
        viz = self.config.viz
        flight = self.config.flight
        
        lines = [
            f"Time: {self.time:.1f}s / {flight.demo_time_s:.1f}s",
            f"Min peer dist: {self.min_peer_distance:.2f}m",
            f"Avoidance events: {self.avoidance_events}",
            "",
        ]
        
        # Add drone status
        for i, (drone, ctrl) in enumerate(zip(self.drones, self.controllers)):
            d0 = drone.distance_to(*self.config.beacon_pos)
            mode_str = ctrl.mode.name
            if ctrl.is_avoiding:
                mode_str = "AVOID"
            state_str = drone.flight_state.name
            lines.append(f"Drone {i+1}: {state_str} | {mode_str} | d0={d0:.2f}m")
        
        lines.append("")
        lines.append("[SPACE] Pause  [R] Reset  [ESC] Quit")
        
        if self.paused:
            lines.insert(0, "== PAUSED ==")
        
        y = 10
        for line in lines:
            text = self.font.render(line, True, viz.text_color)
            self.screen.blit(text, (10, y))
            y += 20


def run_simulation(config: Config = None) -> None:
    """
    Run the simulation with the given configuration.
    
    Args:
        config: Configuration object (uses default if not provided)
    """
    sim = Simulator(config)
    sim.run()
