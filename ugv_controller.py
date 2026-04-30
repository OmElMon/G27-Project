"""
UGV Subsystem for UAV-UGV Coordination System
==============================================
This module implements the UGV (leader) in the leader-follower problem.

RESPONSIBILITIES:
----------------
- Spawn and control the UGV vehicle in CARLA
- Navigate via autopilot or scripted path (using BasicAgent)
- Publish position updates to the Coordination Platform
- Detect when destination is reached

Owner: Sean Bowden
"""

import carla
import random
import time
import math
import sys
import os
import numpy as np
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
from config import UGV_CONFIG, TOPICS, SystemState
from message_broker import MessageBroker, Message, create_position_message, create_status_message


class NavigationMode:
    """Navigation mode options."""
    AUTOPILOT = "autopilot"
    SCRIPTED = "scripted"


class UGVSubsystem:
    """
    UGV Subsystem implementing the leader vehicle.
    
    This class manages:
    1. Vehicle spawning and lifecycle
    2. Navigation (autopilot or scripted path)
    3. Position broadcasting to other subsystems
    4. Destination detection
    
    Usage:
        broker = MessageBroker()
        ugv = UGVSubsystem(world, broker)
        ugv.initialize()
        ugv.set_navigation_mode('scripted', destination=some_location)
        
        In main loop:
        ugv.update()
    """
    
    def __init__(self, world: carla.World, broker: MessageBroker, config: Dict = None):
        """
        Initialize the UGV Subsystem.
        
        Args:
            world: CARLA world object
            broker: MessageBroker for communication
            config: Configuration dictionary (uses UGV_CONFIG defaults if None)
        """
        self.world = world
        self.broker = broker
        self.config = config or UGV_CONFIG
        
        # CARLA objects
        self.vehicle: Optional[carla.Vehicle] = None
        self.lidar: Optional[carla.Sensor] = None
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
        self.last_broadcast_time = 0.0
        self.broadcast_interval = 0.1  # Broadcast position at 10Hz

        # Path marker visualization
        self.show_path_markers = False
        self.marker_color: Optional[carla.Color] = None
        self.marker_lifetime = 2.0 # seconds
        self.last_marker_draw_time = 0.0
        self.marker_redraw_interval = 1.0  # redraw before markers expire
        
        # Status
        self.current_status = SystemState.IDLE
        self.destination_reached = False

        # LIDAR data storage (used for obstacle detection + GUI minimap)
        self.latest_point_cloud: Optional[np.ndarray] = None   # Nx4 array [x, y, z, intensity]
        self.detected_obstacles: List[Dict] = []               # Processed obstacle list
        self.last_obstacle_broadcast_time = 0.0
        self.obstacle_broadcast_interval = 0.2                 # Broadcast obstacles at 5Hz

        print("[UGV] Subsystem created")
    
    def initialize(self, 
                   spawn_point: Optional[carla.Transform] = None,
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

            # === ATTACH LIDAR SENSOR ===
            lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', str(int(self.config.get('lidar_channels', 32))))
            lidar_bp.set_attribute('range', str(self.config.get('lidar_range', 50.0)))
            lidar_bp.set_attribute('points_per_second', str(int(self.config.get('lidar_points_per_second', 300000))))
            lidar_bp.set_attribute('rotation_frequency', str(self.config.get('lidar_rotation_frequency', 20.0)))
            lidar_bp.set_attribute('upper_fov', str(self.config.get('lidar_upper_fov', 10.0)))
            lidar_bp.set_attribute('lower_fov', str(self.config.get('lidar_lower_fov', -30.0)))

            lidar_x = float(self.config.get('lidar_mount_x', 1.0))
            lidar_z = float(self.config.get('lidar_mount_z', 1.4))
            self._lidar_mount_height = lidar_z

            lidar_transform = carla.Transform(
                carla.Location(x=lidar_x, y=0, z=lidar_z),
                carla.Rotation(pitch=0, yaw=0, roll=0)
            )

            self.lidar = self.world.spawn_actor(
                lidar_bp,
                lidar_transform,
                attach_to=self.vehicle
            )
            self.lidar.listen(self._on_lidar_data)
            print("[UGV] LIDAR sensor attached")

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
                print("[UGV] Scripted navigation unavailable: CARLA agents not found")
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
            if hasattr(self.agent, 'set_target_speed'):
                self.agent.set_target_speed(target_speed)
            
            # Configure agent
            self.agent.ignore_traffic_lights(
                self.config.get('ignore_traffic_lights', True))
            self.agent.ignore_stop_signs(
                self.config.get('ignore_stop_signs', True))
            self.agent.follow_speed_limits(
                self.config.get('follow_speed_limits', False))
            
            # Generate path
            grp = GlobalRoutePlanner(self.world.get_map(), 1.0)
            path = grp.trace_route(
                self.vehicle.get_location(),
                self.final_destination
            )
            
            if not path:
                print("[UGV] Failed to generate path")
                return False
            
            self.path_waypoints = path
            self.agent.set_global_plan(path, stop_waypoint_creation=True)
            
            print(f"[UGV] Scripted path created with {len(path)} waypoints")
            print(f"[UGV] Destination: ({destination.x:.1f}, {destination.y:.1f})")
            
            return True
        
        return False
    
    def set_destination(self, destination: carla.Location) -> bool:
        """
        Change the UGV's destination mid-path without recreating the agent.

        Only works in scripted navigation mode. Replaces the current route
        with a new one from the vehicle's current position to the new destination.

        Args:
            destination: New target location

        Returns:
            True if reroute succeeded, False otherwise
        """
        if not self.is_initialized or self.vehicle is None:
            print("[UGV] Cannot set destination - not initialized")
            return False

        if self.navigation_mode != NavigationMode.SCRIPTED or self.agent is None:
            print("[UGV] Cannot set destination - not in scripted mode")
            return False

        return self._route_to(destination, update_final_destination=True)

    def _route_to(self,
                  destination: carla.Location,
                  update_final_destination: bool = False) -> bool:
        """
        Route the BasicAgent to a destination.

        Detours use this without replacing final_destination. User/requested
        reroutes use update_final_destination=True.
        """
        if not self.is_initialized or self.vehicle is None or self.agent is None:
            return False

        if update_final_destination:
            self.final_destination = destination

        self.destination_reached = False

        # BasicAgent.set_destination replaces the current route
        self.agent.set_destination(destination)

        # Regenerate path waypoints for visualization
        grp = GlobalRoutePlanner(self.world.get_map(), 1.0)
        self.path_waypoints = grp.trace_route(
            self.vehicle.get_location(),
            destination
        )

        label = "destination" if update_final_destination else "temporary target"
        print(f"[UGV] Rerouted to {label}: "
              f"({destination.x:.1f}, {destination.y:.1f}, {destination.z:.1f})")
        return True

    def draw_path_markers(self,
                          color: carla.Color = None,
                          enabled: bool = True) -> None:
        """
        Enable or disable continuous path marker drawing.

        Markers use a short lifetime and are periodically redrawn in update().
        When the path changes (e.g., reroute), old markers expire on their own
        and new ones are drawn automatically.

        Args:
            color: Marker color (red by default)
            enabled: True to show markers, False to stop (existing markers fade out)
        """
        self.show_path_markers = enabled
        if enabled:
            self.marker_color = color or carla.Color(r=255, g=0, b=0)
            self._draw_markers()  # Draw immediately
        else:
            print("[UGV] Path markers disabled")

    def _draw_markers(self) -> None:
        """Draw markers for the current path waypoints."""
        if not self.path_waypoints:
            return

        color = self.marker_color or carla.Color(r=255, g=0, b=0)

        for wp, road_option in self.path_waypoints:
            self.world.debug.draw_string(
                wp.transform.location,
                'X',
                color=color,
                life_time=self.marker_lifetime,
                persistent_lines=True
            )
        self.last_marker_draw_time = time.time()
    
    def update(self) -> bool:
        """
        Update the UGV for one simulation tick.
        
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

        # Process LIDAR point cloud into obstacle clusters for downstream
        # subscribers (data logger, GUI minimap).
        self._process_obstacles()

        # Handle navigation based on mode
        if self.navigation_mode == NavigationMode.SCRIPTED and self.agent:
            if self.agent.done():
                self.destination_reached = True
                print("[UGV] Reached final destination!")
                self._publish_status("Destination reached")
                return False

            control = self.agent.run_step()
            self.vehicle.apply_control(control)

        # Broadcast position at configured interval
        if current_time - self.last_broadcast_time >= self.broadcast_interval:
            self._broadcast_position()
            self.last_broadcast_time = current_time

        # Broadcast obstacles at throttled rate (processing already happened above)
        if current_time - self.last_obstacle_broadcast_time >= self.obstacle_broadcast_interval:
            self._broadcast_obstacles()
            self.last_obstacle_broadcast_time = current_time

        # Redraw path markers before they expire
        if self.show_path_markers and current_time - self.last_marker_draw_time >= self.marker_redraw_interval:
            self._draw_markers()

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
    
    def _broadcast_position(self) -> None:
        """Publish current position to the message broker."""
        if self.vehicle is None:
            return
        
        transform = self.vehicle.get_transform()
        location = transform.location
        rotation = transform.rotation
        
        position_data = create_position_message(
            x=location.x,
            y=location.y,
            z=location.z,
            yaw=rotation.yaw,
            velocity=self.current_velocity
        )
        
        self.broker.publish(
            TOPICS['UGV_POSITION'],
            position_data,
            source='UGVSubsystem'
        )
    
    def _on_lidar_data(self, measurement) -> None:
        """
        Callback when LIDAR completes a sweep. Runs on CARLA's sensor thread.

        Args:
            measurement: carla.LidarMeasurement containing the point cloud
        """
        # raw_data is a flat buffer of floats: [x, y, z, intensity, x, y, z, intensity, ...]
        # Each point is 4 floats × 4 bytes = 16 bytes
        points = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4)
        self.latest_point_cloud = points

    def _process_obstacles(self) -> None:
        """
        Process the latest LIDAR point cloud to detect nearby obstacles.

        Steps:
        1. Filter out ground points (below ground_threshold relative to sensor)
        2. Filter to points within the obstacle detection range
        3. Cluster nearby points into distinct obstacles
        4. Publish obstacle data to the message broker
        """
        if self.latest_point_cloud is None or len(self.latest_point_cloud) == 0:
            return

        points = self.latest_point_cloud  # Nx4: [x, y, z, intensity]

        # Ground filtering
        # Points are in sensor-local coordinates. The LIDAR is mounted at z=2.4m
        # above the vehicle base, so ground-level points have z ≈ -2.1 (below sensor).
        # We keep only points above (ground_threshold - sensor_height) in sensor space.
        sensor_height = getattr(
            self, '_lidar_mount_height',
            float(self.config.get('lidar_mount_z', 1.4))
        )
        ground_z = -sensor_height + self.config.get('obstacle_ground_threshold', 0.3)
        above_ground = points[:, 2] > ground_z
        filtered = points[above_ground]

        if len(filtered) == 0:
            self.detected_obstacles = []
            return

        # Distance filtering
        # Compute horizontal distance from sensor to each point
        distances = np.sqrt(filtered[:, 0]**2 + filtered[:, 1]**2)
        threshold = self.config.get('obstacle_distance_threshold', 15.0)
        # Inner cutoff: discard points too close to the LIDAR, these come
        # from the UGV's own body (mirrors, hood, roof rack). Without this,
        # the obstacle list is constantly polluted with a phantom obstacle
        # at ~1 m, especially with low min_points settings.
        min_distance = self.config.get('obstacle_min_distance', 2.0)
        nearby_mask = (distances < threshold) & (distances > min_distance)
        nearby_points = filtered[nearby_mask]
        nearby_distances = distances[nearby_mask]

        if len(nearby_points) == 0:
            self.detected_obstacles = []
            return

        # Simple angular binning for obstacle clustering
        # Divide the 360° around the vehicle into bins and group points by angle.
        # Each non-empty bin with enough points represents one obstacle.
        angles = np.degrees(np.arctan2(nearby_points[:, 1], nearby_points[:, 0]))
        bin_size = 10  # degrees per bin
        bins = ((angles + 180) / bin_size).astype(int) % (360 // bin_size)

        min_points = self.config.get('obstacle_min_points', 5)
        obstacles = []

        for bin_id in np.unique(bins):
            mask = bins == bin_id
            if np.sum(mask) < min_points:
                continue

            cluster_points = nearby_points[mask]
            cluster_distances = nearby_distances[mask]

            # Obstacle represented by its closest point and centroid
            closest_idx = np.argmin(cluster_distances)
            centroid_x = float(np.mean(cluster_points[:, 0]))
            centroid_y = float(np.mean(cluster_points[:, 1]))
            centroid_z = float(np.mean(cluster_points[:, 2]))

            obstacles.append({
                'centroid': {'x': centroid_x, 'y': centroid_y, 'z': centroid_z},
                'distance': float(cluster_distances[closest_idx]),
                'angle': float(np.mean(angles[mask])),
                'point_count': int(np.sum(mask)),
            })

        self.detected_obstacles = obstacles

    def _broadcast_obstacles(self) -> None:
        """Publish detected obstacles to the message broker."""
        obstacle_data = {
            'obstacles': self.detected_obstacles,
            'count': len(self.detected_obstacles),
            'timestamp': time.time(),
        }
        self.broker.publish(
            TOPICS['UGV_OBSTACLES'],
            obstacle_data,
            source='UGVSubsystem'
        )

    def _publish_status(self, message: str) -> None:
        """Publish status update to message broker."""
        status_data = create_status_message(
            state=self.current_status,
            message=message,
            details={
                'destination_reached': self.destination_reached,
                'velocity': self.current_velocity
            }
        )
        self.broker.publish(
            TOPICS['UGV_STATUS'],
            status_data,
            source='UGVSubsystem'
        )
    
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
        Clean up the vehicle actor and sensors.

        IMPORTANT: Always call this when done
        """
        if self.lidar:
            self.lidar.stop()
            self.lidar.destroy()
            print("[UGV] LIDAR destroyed")

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

    Uses interpolation (lerping) to smooth camera movement, avoiding
    jarring snaps when the followed actor turns.
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

        # Smoothing factor: 0.0 = no movement, 1.0 = instant snap
        # Lower values = smoother/slower camera, higher = more responsive
        self.smoothing = config.get('spectator_smoothing', 0.08)

        # Track current smoothed camera state
        self._current_x: Optional[float] = None
        self._current_y: Optional[float] = None
        self._current_z: Optional[float] = None
        self._current_yaw: Optional[float] = None

    @staticmethod
    def _lerp_angle(current: float, target: float, t: float) -> float:
        """
        Interpolate between two angles using the shortest path.

        Handles wrapping around 360 degrees so the camera always
        takes the shortest rotation direction.
        """
        diff = (target - current + 180) % 360 - 180
        return current + diff * t

    def follow_actor(self, actor: carla.Actor) -> None:
        """
        Position spectator to smoothly follow an actor.

        Args:
            actor: The actor to follow
        """
        if actor is None:
            return

        transform = actor.get_transform()
        forward = transform.get_forward_vector()

        # Calculate desired camera position behind and above the actor
        target_x = transform.location.x - self.follow_distance * forward.x
        target_y = transform.location.y - self.follow_distance * forward.y
        target_z = transform.location.z + self.follow_height
        target_yaw = transform.rotation.yaw

        # Initialize on first call (snap to position immediately)
        if self._current_x is None:
            self._current_x = target_x
            self._current_y = target_y
            self._current_z = target_z
            self._current_yaw = target_yaw
        else:
            # Smoothly interpolate position and rotation
            t = self.smoothing
            self._current_x += (target_x - self._current_x) * t
            self._current_y += (target_y - self._current_y) * t
            self._current_z += (target_z - self._current_z) * t
            self._current_yaw = self._lerp_angle(self._current_yaw, target_yaw, t)

        spectator_transform = carla.Transform(
            carla.Location(x=self._current_x, y=self._current_y, z=self._current_z),
            carla.Rotation(pitch=self.follow_pitch, yaw=self._current_yaw, roll=0)
        )

        self.spectator.set_transform(spectator_transform)

    def follow_position(self, x: float, y: float, z: float, yaw: float = 0) -> None:
        """
        Position spectator to smoothly look at a specific position.

        Args:
            x, y, z: Position to look at
            yaw: Direction the camera should face
        """
        yaw_rad = math.radians(yaw)
        forward_x = math.cos(yaw_rad)
        forward_y = math.sin(yaw_rad)

        target_x = x - self.follow_distance * forward_x
        target_y = y - self.follow_distance * forward_y
        target_z = z + self.follow_height

        # Initialize on first call
        if self._current_x is None:
            self._current_x = target_x
            self._current_y = target_y
            self._current_z = target_z
            self._current_yaw = yaw
        else:
            t = self.smoothing
            self._current_x += (target_x - self._current_x) * t
            self._current_y += (target_y - self._current_y) * t
            self._current_z += (target_z - self._current_z) * t
            self._current_yaw = self._lerp_angle(self._current_yaw, yaw, t)

        spectator_transform = carla.Transform(
            carla.Location(x=self._current_x, y=self._current_y, z=self._current_z),
            carla.Rotation(pitch=self.follow_pitch, yaw=self._current_yaw, roll=0)
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
# STANDALONE TEST
# =============================================================================
if __name__ == '__main__':
    """
    Test the UGV subsystem independently.
    
    This recreates the behavior of the original Initial_UGV.py script.
    """
    print("Testing UGV Subsystem...")
    print("Make sure CARLA server is running")
    
    ugv = None
    
    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        print("Connected to CARLA")
        
        # Create message broker (for testing, we'll just print received messages)
        broker = MessageBroker()
        
        # Subscribe to UGV position to verify publishing works
        def on_ugv_position(msg):
            data = msg.data
            print(f"[Broker] UGV Position: ({data['x']:.1f}, {data['y']:.1f}) "
                  f"vel={data['velocity']:.1f} m/s")
        broker.subscribe(TOPICS['UGV_POSITION'], on_ugv_position)
        
        # Create UGV subsystem
        ugv = UGVSubsystem(world, broker)
        
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
