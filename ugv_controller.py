'''
This module implements the UGV (leader) in the leader-follower problem.

This is a refactored version of Initial_UGV.py, converted to a class structure.

RESPONSIBILITIES:
- Spawn and control the UGV vehicle in CARLA
- Navigate via autopilot or scripted path (using BasicAgent)
- Publish position updates to the Coordination Platform
- Detect when destination is reached

TO DOS:
- Decide on location publishing and implement it


Owner: Sean Bowden
''' 

import carla
import random
import time
import math
import sys
import os
from typing import Optional, Tuple, Dict, List, Any

# Import CARLA agents (adjust path as needed for your setup)
# This assumes the Carla folder is in the same directory
try:
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    agent_dir = os.path.join(curr_dir, 'Carla/PythonAPI/carla')
    if os.path.exists(agent_dir):
        sys.path.append(agent_dir)
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    AGENTS_AVAILABLE = True
except ImportError:
    print("[UGV] Warning: CARLA agents not found. Scripted path navigation unavailable.")
    AGENTS_AVAILABLE = False

# Import our modules
from config import UGV_CONFIG, SystemState

class NavigationMode:
    """Navigation mode options."""
    AUTOPILOT = "autopilot"
    SCRIPTED = "scripted"
    MANUAL = "manual"


class UGVSubsystem:
    """
    UGV Subsystem implementing the leader vehicle.
    
    This class manages:
    1. Vehicle spawning and lifecycle
    2. Navigation (autopilot or scripted path)
    3. Position broadcasting to other subsystems
    4. Destination detection
    
    Usage:
        ugv = UGVSubsystem(world)
        ugv.initialize()
        ugv.set_navigation_mode('scripted', destination=some_location)
        
        # In main loop:
        ugv.update()
    """
    
    def __init__(self, world: carla.World, config: Dict = None):
        """
        Initialize the UGV Subsystem.
        
        Args:
            world: CARLA world object
            config: Configuration dictionary (uses UGV_CONFIG defaults if None)
        """
        self.world = world
        self.config = config or UGV_CONFIG
        
        # CARLA objects
        self.vehicle: Optional[carla.Vehicle] = None
        self.agent: Optional['BasicAgent'] = None
        
        # State tracking
        self.is_initialized = False
        self.navigation_mode = NavigationMode.AUTOPILOT
        self.final_destination: Optional[carla.Location] = None
        self.path_waypoints: List = []
        
        # Position tracking for velocity calculation
        self.last_position: Optional[Tuple[float, float, float]] = None
        self.last_position_time: float = 0.0
        self.current_velocity: float = 0.0
        
        # Timing
        self.last_update_time = time.time()
        
        # Status
        self.current_status = SystemState.IDLE
        self.destination_reached = False
        
        print("[UGV] Subsystem created")
    
    def initialize(self, spawn_point: Optional[carla.Transform] = None,
                   blueprint_filter: str = None) -> bool:
        """
        Initialize and spawn the UGV vehicle.
        
        Args:
            spawn_point: Where to spawn (random if None)
            blueprint_filter: Vehicle blueprint filter (uses config default if None)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            bp_lib = self.world.get_blueprint_library()
            
            # Select vehicle blueprint
            filter_str = blueprint_filter or self.config.get('blueprint', 'vehicle.*')
            vehicle_bps = bp_lib.filter(filter_str)
            
            if len(vehicle_bps) == 0:
                # Fall back to any vehicle if specific one not found
                vehicle_bps = bp_lib.filter('vehicle.*')
            
            ugv_bp = random.choice(vehicle_bps)
            
            # Get spawn point
            spawn_points = self.world.get_map().get_spawn_points()
            if spawn_point is None:
                spawn_point = random.choice(spawn_points)
            
            # Spawn the vehicle
            self.vehicle = self.world.spawn_actor(ugv_bp, spawn_point)
            
            if self.vehicle is None:
                print("[UGV] Failed to spawn vehicle")
                return False
            
            print(f"[UGV] Spawned: {self.vehicle.type_id}")
            print(f"[UGV] Location: ({spawn_point.location.x:.1f}, "
                  f"{spawn_point.location.y:.1f}, {spawn_point.location.z:.1f})")
            
            # Initialize position tracking
            loc = self.vehicle.get_location()
            self.last_position = (loc.x, loc.y, loc.z)
            self.last_position_time = time.time()
            
            self.is_initialized = True
            self.current_status = SystemState.TRACKING
            
            return True
            
        except Exception as e:
            print(f"[UGV] Initialization failed: {e}")
            self.current_status = SystemState.ERROR
            return False
    
    def set_navigation_mode(self, 
                            mode: str,
                            destination: Optional[carla.Location] = None,
                            target_speed: float = None) -> bool:
        """
        Set the navigation mode for the UGV.
        
        Args:
            mode: 'autopilot' or 'scripted'
            destination: Required for 'scripted' mode
            target_speed: Desired speed in km/h (uses config default if None)
            
        Returns:
            True if successful, False otherwise
        """
        if not self.is_initialized:
            print("[UGV] Cannot set navigation mode - not initialized")
            return False
        
        self.navigation_mode = mode
        target_speed = target_speed or self.config.get('target_speed', 30.0)
        
        if mode == NavigationMode.AUTOPILOT:
            # Use CARLA's built-in autopilot
            self.vehicle.set_autopilot(True, 8000)  # Port 8000 for Traffic Manager
            print(f"[UGV] Autopilot enabled")
            return True
            
        elif mode == NavigationMode.SCRIPTED:
            if not AGENTS_AVAILABLE:
                print("[UGV] Scripted navigation unavailable - CARLA agents not found")
                print("[UGV] Falling back to autopilot")
                return self.set_navigation_mode(NavigationMode.AUTOPILOT)
            
            if destination is None:
                # Pick a random destination
                spawn_points = self.world.get_map().get_spawn_points()
                current_loc = self.vehicle.get_location()
                # Filter out points too close
                valid_destinations = [
                    p for p in spawn_points 
                    if p.location.distance(current_loc) > 50.0
                ]
                if valid_destinations:
                    destination = random.choice(valid_destinations).location
                else:
                    destination = random.choice(spawn_points).location
            
            self.final_destination = destination
            
            # Initialize BasicAgent
            self.agent = BasicAgent(self.vehicle)
            
            # Configure agent
            self.agent.ignore_traffic_lights(
                self.config.get('ignore_traffic_lights', True))
            self.agent.ignore_stop_signs(
                self.config.get('ignore_stop_signs', True))
            self.agent.follow_speed_limits(
                self.config.get('follow_speed_limits', False))
            
            # Generate path
            grp = GlobalRoutePlanner(self.world.get_map(), 1.0)
            path = grp.trace_route(self.vehicle.get_location(),
                                   self.final_destination)
            
            if not path:
                print("[UGV] Failed to generate path")
                return False
            
            self.path_waypoints = path
            self.agent.set_global_plan(path, stop_waypoint_creation=True)
            
            print(f"[UGV] Scripted path created with {len(path)} waypoints")
            print(f"[UGV] Destination: ({destination.x:.1f}, {destination.y:.1f})")
            
            return True
            
        elif mode == NavigationMode.MANUAL:
            # Disable autopilot for manual control
            self.vehicle.set_autopilot(False)
            print("[UGV] Manual mode - vehicle ready for external control")
            return True
        
        return False
    
    def draw_path_markers(self, color: carla.Color = None,
                          lifetime: float = 90.0) -> None:
        """
        Draw path markers in the CARLA world for visualization.
        
        Args:
            color: Marker color (red by default)
            lifetime: How long markers persist (seconds)
        """
        if not self.path_waypoints:
            print("[UGV] No path to draw")
            return
        
        color = color or carla.Color(r=255, g=0, b=0)
        
        for wp, road_option in self.path_waypoints:
            self.world.debug.draw_string(
                wp.transform.location,
                'X',
                color=color,
                life_time=lifetime,
                persistent_lines=True
            )
        
        print(f"[UGV] Drew {len(self.path_waypoints)} path markers")
    
    def update(self) -> bool:
        """
        Update the UGV for one simulation tick.
        
        This should be called every tick from your main loop.
        
        Returns:
            True if update successful, False if destination reached or error
        """
        if not self.is_initialized or self.vehicle is None:
            return False
        
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update velocity calculation
        self._update_velocity(dt)
        
        # Handle navigation based on mode
        if self.navigation_mode == NavigationMode.SCRIPTED and self.agent:
            # Check if destination reached
            if self.agent.done():
                self.destination_reached = True
                print("[UGV] Reached final destination!")
                return False
            
            # Get and apply control from agent
            control = self.agent.run_step()
            self.vehicle.apply_control(control)
        
        return True
    
    def _update_velocity(self, dt: float) -> None:
        """Calculate current velocity from position change."""
        if self.vehicle is None:
            return
        
        loc = self.vehicle.get_location()
        current_pos = (loc.x, loc.y, loc.z)
        
        if self.last_position is not None and dt > 0:
            dx = current_pos[0] - self.last_position[0]
            dy = current_pos[1] - self.last_position[1]
            dz = current_pos[2] - self.last_position[2]
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            self.current_velocity = distance / dt
        
        self.last_position = current_pos
    
    def get_location(self) -> Optional[carla.Location]:
        """Get current vehicle location."""
        if self.vehicle:
            return self.vehicle.get_location()
        return None
    
    def get_transform(self) -> Optional[carla.Transform]:
        """Get current vehicle transform (location + rotation)."""
        if self.vehicle:
            return self.vehicle.get_transform()
        return None
    
    def get_velocity(self) -> float:
        """Get current velocity in m/s."""
        return self.current_velocity
    
    def get_destination(self) -> Optional[carla.Location]:
        """Get the final destination (if in scripted mode)."""
        return self.final_destination
    
    def has_reached_destination(self) -> bool:
        """Check if UGV has reached its destination."""
        return self.destination_reached
    
    def cleanup(self) -> None:
        """
        Clean up the vehicle actor.
        
        IMPORTANT: Always call this when done
        """
        if self.vehicle:
            # Disable autopilot first
            try:
                self.vehicle.set_autopilot(False)
            except:
                pass
            
            # Destroy the actor
            self.vehicle.destroy()
            print("[UGV] Vehicle destroyed")
        
        self.vehicle = None
        self.agent = None
        self.is_initialized = False


class SpectatorController:
    """
    Helper class to manage the spectator camera.
    
    This is separated from the UGV subsystem to allow flexibility
    in what the camera follows (UGV, UAV, or custom position).
    """
    
    def __init__(self, world: carla.World, config: Dict = None):
        """
        Initialize spectator controller.
        
        Args:
            world: CARLA world object
            config: UI configuration dictionary
        """
        self.world = world
        self.spectator = world.get_spectator()
        
        # Default follow parameters
        from config import UI_CONFIG
        config = config or UI_CONFIG
        self.follow_distance = config.get('spectator_distance', 20.0)
        self.follow_height = config.get('spectator_height', 15.0)
        self.follow_pitch = config.get('spectator_pitch', -30.0)
    
    def follow_actor(self, actor: carla.Actor) -> None:
        """
        Position spectator to follow an actor.
        
        Args:
            actor: The actor to follow
        """
        if actor is None:
            return
        
        transform = actor.get_transform()
        forward = transform.get_forward_vector()
        
        # Calculate camera position behind and above the actor
        offset = carla.Location(
            x=-self.follow_distance * forward.x,
            y=-self.follow_distance * forward.y,
            z=self.follow_height
        )
        
        spectator_transform = carla.Transform(
            transform.location + offset,
            carla.Rotation(
                pitch=self.follow_pitch,
                yaw=transform.rotation.yaw,
                roll=0
            )
        )
        
        self.spectator.set_transform(spectator_transform)
    
    def follow_position(self, x: float, y: float, z: float, yaw: float = 0) -> None:
        """
        Position spectator to look at a specific position.
        
        Args:
            x, y, z: Position to look at
            yaw: Direction the camera should face
        """
        yaw_rad = math.radians(yaw)
        forward_x = math.cos(yaw_rad)
        forward_y = math.sin(yaw_rad)
        
        spectator_transform = carla.Transform(
            carla.Location(
                x=x - self.follow_distance * forward_x,
                y=y - self.follow_distance * forward_y,
                z=z + self.follow_height
            ),
            carla.Rotation(
                pitch=self.follow_pitch,
                yaw=yaw,
                roll=0
            )
        )
        
        self.spectator.set_transform(spectator_transform)
    
    def set_follow_parameters(self, 
                              distance: float = None,
                              height: float = None,
                              pitch: float = None) -> None:
        """Update follow parameters."""
        if distance is not None:
            self.follow_distance = distance
        if height is not None:
            self.follow_height = height
        if pitch is not None:
            self.follow_pitch = pitch


# =============================================================================
# STANDALONE TEST (similar to original Initial_UGV.py)
# =============================================================================
if __name__ == '__main__':
    """
    Test the UGV subsystem independently.
    
    This recreates the behavior of the original Initial_UGV.py script.
    """
    print("Testing UGV Subsystem...")
    print("Make sure CARLA server is running!")
    
    ugv = None
    
    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        print("Connected to CARLA")
        
        # Create UGV subsystem
        ugv = UGVSubsystem(world)
        
        # Initialize
        if not ugv.initialize():
            print("Failed to initialize UGV")
            sys.exit(1)
        
        # Create spectator controller
        spectator = SpectatorController(world)
        
        # Let user choose navigation mode
        print('\nSelect Navigation Mode:')
        print('1: Random Autopilot')
        print('2: Scripted Path')
        choice = input('Enter choice (1 or 2): ').strip()
        
        if choice == '2':
            ugv.set_navigation_mode(NavigationMode.SCRIPTED)
            ugv.draw_path_markers()
        else:
            ugv.set_navigation_mode(NavigationMode.AUTOPILOT)
        
        print('\nSimulation running. Press Ctrl+C to stop.\n')
        
        # Main loop
        while True:
            world.wait_for_tick()
            
            # Update UGV
            if not ugv.update():
                print("UGV reached destination or error occurred")
                break
            
            # Update spectator camera
            spectator.follow_actor(ugv.vehicle)
            
    except KeyboardInterrupt:
        print('\nStopping simulation...')
    finally:
        if ugv:
            ugv.cleanup()
        print('Test complete!')