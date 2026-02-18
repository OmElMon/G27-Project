"""
UAV Subsystem for UAV-UGV Coordination System
==============================================
This module implements the UAV (follower) in the leader-follower problem.

ARCHITECTURE OVERVIEW:
---------------------
Since CARLA doesn't have native UAV support, we implement the UAV as:
1. An "invisible" actor (sensor platform) OR a static prop with disabled physics
2. A kinematic controller that simulates realistic flight behavior
3. Attached sensors (RGB camera) for visual tracking

The UAV moves smoothly with velocity and acceleration constraints, making it 
possible to actually "lose" the target if the UGV moves too fast or erratically.

KINEMATIC MODEL:
---------------
We use a simple kinematic model:
- Position updates based on velocity
- Velocity changes limited by max acceleration
- Speed capped at max velocity
- Separate limits for horizontal and vertical movement

INTEGRATION:
-----------
The UAV Subsystem:
- Subscribes to: 'coordination/uav_waypoint' (where to go)
- Publishes to: 'uav/position' (where it currently is)
- Publishes to: 'uav/status' (current state)

Owner: Sean Bowden
"""

import carla
import math
import time
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass

from config import UAV_CONFIG, TOPICS, SystemState
from message_broker import MessageBroker, Message, create_position_message, create_status_message


@dataclass
class UAVState:
    """
    Represents the complete state of the UAV at a point in time.
    
    We track both position and velocity to implement smooth motion.
    """
    # Position in CARLA world coordinates (meters)
    x: float = 0.0
    y: float = 0.0
    z: float = 30.0  # Default altitude
    
    # Velocity components (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    
    # Orientation (degrees)
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    
    def get_speed(self) -> float:
        """Calculate total speed from velocity components."""
        return math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
    
    def get_horizontal_speed(self) -> float:
        """Calculate horizontal speed."""
        return math.sqrt(self.vx**2 + self.vy**2)
    
    def get_position(self) -> Tuple[float, float, float]:
        """Return position as tuple."""
        return (self.x, self.y, self.z)
    
    def distance_to(self, x: float, y: float, z: float) -> float:
        """Calculate 3D distance to a point."""
        return math.sqrt((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2)
    
    def horizontal_distance_to(self, x: float, y: float) -> float:
        """Calculate horizontal distance to a point."""
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)


class UAVController:
    """
    Kinematic controller for smooth UAV movement.
    
    This controller takes a target position and generates smooth motion
    toward it, respecting velocity and acceleration limits.
    
    The control algorithm:
    1. Calculate direction to target
    2. Calculate desired velocity (toward target, capped at max speed)
    3. Limit velocity change by max acceleration
    4. Update position based on velocity
    5. Update orientation to face direction of travel
    """
    
    def __init__(self, config: Dict = None):
        """
        Initialize the UAV controller.
        
        Args:
            config: Configuration dictionary (uses UAV_CONFIG defaults if None)
        """
        self.config = config or UAV_CONFIG
        
        # Extract limits from config
        self.max_speed = self.config['max_speed']
        self.max_accel = self.config['max_acceleration']
        self.max_vertical_speed = self.config['max_vertical_speed']
        self.max_yaw_rate = self.config['max_yaw_rate']
        self.position_tolerance = self.config['position_tolerance']
        
        # Current state
        self.state = UAVState()
        
        # Target position
        self.target_x: float = 0.0
        self.target_y: float = 0.0
        self.target_z: float = self.config['default_altitude']
        self.target_yaw: Optional[float] = None  # None = face direction of travel
        
        # For tracking
        self.last_update_time: float = time.time()
        self.is_at_target: bool = False
    
    def set_target(self, x: float, y: float, z: float, yaw: Optional[float] = None) -> None:
        """
        Set a new target position for the UAV.
        
        Args:
            x, y, z: Target position in world coordinates
            yaw: Desired heading at target (None = face direction of travel)
        """
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw
        self.is_at_target = False
    
    def set_position(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        """
        Directly set UAV position (used for initialization).
        
        Args:
            x, y, z: New position
            yaw: New heading
        """
        self.state.x = x
        self.state.y = y
        self.state.z = z
        self.state.yaw = yaw
        # Reset velocity when teleporting
        self.state.vx = 0.0
        self.state.vy = 0.0
        self.state.vz = 0.0
    
    def update(self, dt: float) -> UAVState:
        """
        Update UAV state for one time step.
        
        Args:
            dt: Time step in seconds (time since last update)
            
        Returns:
            Updated UAVState
        """
        # Safety check for large time steps (e.g., if simulation was paused)
        dt = min(dt, 0.1)  # Cap at 100ms to prevent huge jumps
        
        # Calculate vector to target
        dx = self.target_x - self.state.x
        dy = self.target_y - self.state.y
        dz = self.target_z - self.state.z
        
        # Calculate distances
        horizontal_dist = math.sqrt(dx**2 + dy**2)
        vertical_dist = abs(dz)
        total_dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Check if we're at the target
        if total_dist < self.position_tolerance:
            self.is_at_target = True
            # Gradually slow down when at target
            self.state.vx *= 0.9
            self.state.vy *= 0.9
            self.state.vz *= 0.9
        else:
            self.is_at_target = False
            
            # === HORIZONTAL MOVEMENT ===
            if horizontal_dist > 0.1:  # Avoid division by zero
                # Normalize direction
                dir_x = dx / horizontal_dist
                dir_y = dy / horizontal_dist
                
                # Calculate desired speed based on distance
                # Slow down as we approach target
                desired_horizontal_speed = min(
                    self.max_speed,
                    horizontal_dist * 2.0  # Gain factor - adjust for responsiveness
                )
                
                # Desired velocity
                desired_vx = dir_x * desired_horizontal_speed
                desired_vy = dir_y * desired_horizontal_speed
                
                # Calculate required acceleration
                accel_x = (desired_vx - self.state.vx) / dt
                accel_y = (desired_vy - self.state.vy) / dt
                
                # Limit acceleration
                accel_magnitude = math.sqrt(accel_x**2 + accel_y**2)
                if accel_magnitude > self.max_accel:
                    accel_x = accel_x / accel_magnitude * self.max_accel
                    accel_y = accel_y / accel_magnitude * self.max_accel
                
                # Apply acceleration to velocity
                self.state.vx += accel_x * dt
                self.state.vy += accel_y * dt
                
                # Cap horizontal speed
                current_h_speed = self.state.get_horizontal_speed()
                if current_h_speed > self.max_speed:
                    scale = self.max_speed / current_h_speed
                    self.state.vx *= scale
                    self.state.vy *= scale
            
            # === VERTICAL MOVEMENT ===
            if vertical_dist > 0.1:
                # Desired vertical velocity
                desired_vz = math.copysign(
                    min(self.max_vertical_speed, vertical_dist * 2.0),
                    dz
                )
                
                # Simple proportional control for vertical
                self.state.vz += (desired_vz - self.state.vz) * min(1.0, dt * 5.0)
                
                # Cap vertical speed
                self.state.vz = max(-self.max_vertical_speed, 
                                   min(self.max_vertical_speed, self.state.vz))
        
        # === UPDATE POSITION ===
        self.state.x += self.state.vx * dt
        self.state.y += self.state.vy * dt
        self.state.z += self.state.vz * dt
        
        # === UPDATE ORIENTATION ===
        # Yaw: face direction of travel
        if self.target_yaw is not None:
            desired_yaw = self.target_yaw
        elif self.state.get_horizontal_speed() > 0.5:  # Only update if moving
            desired_yaw = math.degrees(math.atan2(self.state.vy, self.state.vx))
        else:
            desired_yaw = self.state.yaw  # Keep current heading
        
        # Smooth yaw rotation with rate limit
        yaw_diff = (desired_yaw - self.state.yaw + 180) % 360 - 180  # Shortest path
        max_yaw_change = self.max_yaw_rate * dt
        yaw_change = max(-max_yaw_change, min(max_yaw_change, yaw_diff))
        self.state.yaw += yaw_change
        
        # Pitch: tilt based on forward velocity (visual effect)
        forward_speed = self.state.get_horizontal_speed()
        self.state.pitch = -min(15, forward_speed * 2)  # Tilt forward when moving
        
        return self.state
    
    def get_distance_to_target(self) -> float:
        """Get current distance to target."""
        return self.state.distance_to(self.target_x, self.target_y, self.target_z)


class UAVSubsystem:
    """
    Complete UAV Subsystem integrating CARLA actor and kinematic controller.
    
    This class:
    1. Spawns/manages the UAV actor in CARLA
    2. Runs the kinematic controller
    3. Handles communication via the message broker
    4. Manages sensors (camera)
    
    Usage:
        broker = MessageBroker()
        uav = UAVSubsystem(world, broker)
        uav.initialize(start_x, start_y, start_z)
        
        In main loop:
        uav.update()
    """
    
    def __init__(self, world: carla.World, broker: MessageBroker, config: Dict = None):
        """
        Initialize the UAV Subsystem.
        
        Args:
            world: CARLA world object
            broker: MessageBroker for communication
            config: Configuration dictionary (uses UAV_CONFIG defaults if None)
        """
        self.world = world
        self.broker = broker
        self.config = config or UAV_CONFIG
        
        # Initialize controller
        self.controller = UAVController(self.config)
        
        # CARLA actors (initialized in initialize())
        self.actor: Optional[carla.Actor] = None
        self.camera: Optional[carla.Sensor] = None
        
        # State tracking
        self.is_initialized = False
        self.last_update_time = time.time()
        self.current_status = SystemState.IDLE
        
        # Subscribe to waypoint commands from coordination platform
        self.broker.subscribe(TOPICS['UAV_WAYPOINT'], self._on_waypoint_received)
        
        # Camera image storage (most recent frame)
        self.latest_camera_image = None
    
    def initialize(self, x: float, y: float, z: float, yaw: float = 0.0) -> bool:
        """
        Initialize the UAV at a starting position.
        
        This spawns the CARLA actor and sets up sensors.
        
        Args:
            x, y, z: Starting position
            yaw: Starting heading (degrees)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            bp_lib = self.world.get_blueprint_library()
            
            # === SPAWN UAV ACTOR ===
            # TODO: make custom mesh for uav later (if time permits)
            uav_bp = None
            prop_options = [
                'static.prop.warningaccident',
                'static.prop.box01',
            ]
            
            for prop_name in prop_options:
                try:
                    uav_bp = bp_lib.find(prop_name)
                    break
                except:
                    continue
            
            # If no prop found, we'll create a sensor-only platform (invisible UAV)
            if uav_bp is None:
                print("[UAV] Warning: No suitable prop found. UAV will be invisible (sensor-only).")
                self.actor = None
            else:
                # Spawn the prop at the starting position
                spawn_transform = carla.Transform(
                    carla.Location(x=x, y=y, z=z),
                    carla.Rotation(pitch=0, yaw=yaw, roll=0)
                )
                
                self.actor = self.world.spawn_actor(uav_bp, spawn_transform)
                
                # Disable physics so it doesn't fall
                self.actor.set_simulate_physics(False)
                
                print(f"[UAV] Spawned actor: {self.actor.type_id} at ({x:.1f}, {y:.1f}, {z:.1f})")
            
            # === ATTACH CAMERA SENSOR ===
            camera_bp = bp_lib.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '640')
            camera_bp.set_attribute('image_size_y', '480')
            camera_bp.set_attribute('fov', '90')
            
            # Camera transform relative to UAV (pointing down and forward)
            camera_transform = carla.Transform(
                carla.Location(x=0, y=0, z=-1),  # Slightly below UAV
                carla.Rotation(pitch=-45, yaw=0, roll=0)  # Angled down
            )
            
            if self.actor:
                # Attach to actor
                self.camera = self.world.spawn_actor(
                    camera_bp, 
                    camera_transform, 
                    attach_to=self.actor
                )
            else:
                # Spawn camera at UAV position
                camera_transform.location = carla.Location(x=x, y=y, z=z-1)
                self.camera = self.world.spawn_actor(camera_bp, camera_transform)
            
            # Set up camera callback
            self.camera.listen(self._on_camera_image)
            print("[UAV] Camera sensor attached")
            
            # === INITIALIZE CONTROLLER ===
            self.controller.set_position(x, y, z, yaw)
            self.controller.set_target(x, y, z, yaw)  # Start hovering in place
            
            self.is_initialized = True
            self.current_status = SystemState.TRACKING
            self.last_update_time = time.time()
            
            # Publish initial status
            self._publish_status("UAV initialized and ready")
            
            return True
            
        except Exception as e:
            print(f"[UAV] Initialization failed: {e}")
            self.current_status = SystemState.ERROR
            return False
    
    def update(self) -> None:
        """
        Update the UAV state for one tick.
        """
        if not self.is_initialized:
            return
        
        # Calculate time step
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update kinematic controller
        state = self.controller.update(dt)
        
        # Move CARLA actor to new position
        if self.actor:
            new_transform = carla.Transform(
                carla.Location(x=state.x, y=state.y, z=state.z),
                carla.Rotation(pitch=state.pitch, yaw=state.yaw, roll=state.roll)
            )
            self.actor.set_transform(new_transform)
        
        # If no actor but we have a camera, move the camera
        elif self.camera:
            camera_transform = carla.Transform(
                carla.Location(x=state.x, y=state.y, z=state.z - 1),
                carla.Rotation(pitch=-45, yaw=state.yaw, roll=0)
            )
            self.camera.set_transform(camera_transform)
        
        # Publish current position
        self._publish_position()
    
    def set_target(self, x: float, y: float, z: float, yaw: Optional[float] = None) -> None:
        """
        Set a new target position for the UAV.
        
        This can be called directly or will be called automatically
        when a waypoint message is received.
        
        Args:
            x, y, z: Target position
            yaw: Desired heading (None = face direction of travel)
        """
        self.controller.set_target(x, y, z, yaw)
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current UAV position."""
        return self.controller.state.get_position()
    
    def get_state(self) -> UAVState:
        """Get complete UAV state."""
        return self.controller.state
    
    def get_distance_to_target(self) -> float:
        """Get distance to current target."""
        return self.controller.get_distance_to_target()
    
    def is_at_target(self) -> bool:
        """Check if UAV has reached its target."""
        return self.controller.is_at_target
    
    def _on_waypoint_received(self, message: Message) -> None:
        """
        Callback when a new waypoint is received from Coordination Platform.
        
        This is automatically called by the message broker when a message
        is published to the UAV_WAYPOINT topic.
        """
        data = message.data
        x = data.get('x', self.controller.target_x)
        y = data.get('y', self.controller.target_y)
        z = data.get('z', self.controller.target_z)
        yaw = data.get('yaw', None)
        
        self.set_target(x, y, z, yaw)
    
    def _on_camera_image(self, image: carla.Image) -> None:
        """
        Callback when camera captures a new frame.
        """
        self.latest_camera_image = image
    
    def _publish_position(self) -> None:
        """Publish current position to the message broker."""
        state = self.controller.state
        position_data = create_position_message(
            x=state.x,
            y=state.y,
            z=state.z,
            yaw=state.yaw,
            velocity=state.get_speed()
        )
        self.broker.publish(TOPICS['UAV_POSITION'], position_data, source='UAVSubsystem')
    
    def _publish_status(self, message: str) -> None:
        """Publish status update to the message broker."""
        status_data = create_status_message(
            state=self.current_status,
            message=message,
            details={
                'distance_to_target': self.controller.get_distance_to_target(),
                'speed': self.controller.state.get_speed()
            }
        )
        self.broker.publish(TOPICS['UAV_STATUS'], status_data, source='UAVSubsystem')
    
    def cleanup(self) -> None:
        """
        Clean up CARLA actors.
        
        IMPORTANT: Always call this when done to prevent actors
        from persisting in the simulation
        """
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            print("[UAV] Camera destroyed")
        
        if self.actor:
            self.actor.destroy()
            print("[UAV] Actor destroyed")
        
        self.is_initialized = False


# =============================================================================
# STANDALONE TEST
# =============================================================================
if __name__ == '__main__':
    """
    Test the UAV subsystem independently.
    
    This spawns a UAV and moves it to a series of waypoints.
    Run this with CARLA server running to test.
    """
    import sys
    
    print("Testing UAV Subsystem...")
    
    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        print("Connected to CARLA")
        
        # Create message broker
        broker = MessageBroker()
        
        # Create and initialize UAV
        uav = UAVSubsystem(world, broker)
        
        spawn_points = world.get_map().get_spawn_points()
        start = spawn_points[0].location
        
        # Initialize UAV above the spawn point
        uav.initialize(start.x, start.y, start.z + 30)
        
        # Get spectator to watch the UAV
        spectator = world.get_spectator()
        
        # Define some test waypoints
        waypoints = [
            (start.x + 50, start.y, start.z + 30),
            (start.x + 50, start.y + 50, start.z + 40),
            (start.x, start.y + 50, start.z + 35),
            (start.x, start.y, start.z + 30),
        ]
        waypoint_idx = 0
        uav.set_target(*waypoints[waypoint_idx])
        
        print(f"\nMoving to waypoint {waypoint_idx + 1}: {waypoints[waypoint_idx]}")
        print("Press Ctrl+C to stop\n")
        
        # Main loop
        last_print_time = time.time()
        while True:
            world.wait_for_tick()
            uav.update()
            
            # Check if reached waypoint
            if uav.is_at_target():
                waypoint_idx = (waypoint_idx + 1) % len(waypoints)
                uav.set_target(*waypoints[waypoint_idx])
                print(f"Moving to waypoint {waypoint_idx + 1}: {waypoints[waypoint_idx]}")
            
            # Update spectator to follow UAV
            pos = uav.get_position()
            spec_transform = carla.Transform(
                carla.Location(x=pos[0] - 30, y=pos[1], z=pos[2] + 20),
                carla.Rotation(pitch=-30, yaw=0, roll=0)
            )
            spectator.set_transform(spec_transform)
            
            # Print status periodically
            if time.time() - last_print_time > 2.0:
                state = uav.get_state()
                print(f"UAV: pos=({state.x:.1f}, {state.y:.1f}, {state.z:.1f}) "
                      f"speed={state.get_speed():.1f} m/s "
                      f"dist_to_target={uav.get_distance_to_target():.1f} m")
                last_print_time = time.time()
                
    except KeyboardInterrupt:
        print("\nStopping test...")
    finally:
        if 'uav' in locals():
            uav.cleanup()
        print("Test complete!")
