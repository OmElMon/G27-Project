"""
Coordination Platform for UAV-UGV Coordination System
======================================================
This module implements the core leader-follower coordination logic.

State Transitions:
- IDLE -> INITIALIZING: Operator starts simulation
- INITIALIZING -> TRACKING: Vehicles spawned and ready
- TRACKING -> TARGET_LOST: UGV position not received for >3 seconds
- TARGET_LOST -> TRACKING: UGV position reacquired
- TARGET_LOST -> SIMULATION_COMPLETE: Position not reacquired in 60 seconds
- TRACKING -> SIMULATION_COMPLETE: UGV reaches final destination
- Any -> ERROR: Critical fault detected

WAYPOINT CALCULATION:
--------------------
The platform calculates where the UAV should be based on:
1. Current UGV position
2. UGV velocity/heading (for predictive following)
3. Configured follow distance and altitude
4. Optional: Path prediction for smoother following
"""

import math
import time
from typing import Optional, Tuple, Dict, Any, Callable
from dataclasses import dataclass
from enum import Enum

from config import (
    COORDINATION_CONFIG, UAV_CONFIG, TOPICS, SystemState
)
from message_broker import (
    MessageBroker, Message, 
    create_waypoint_message, create_status_message
)


@dataclass
class VehicleState:
    """Stores the last known state of a vehicle (UGV or UAV)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    velocity: float = 0.0
    timestamp: float = 0.0
    
    def age(self) -> float:
        """Return how old this state information is."""
        return time.time() - self.timestamp
    
    def get_forward_vector(self) -> Tuple[float, float]:
        """Get unit vector in the direction the vehicle is facing."""
        rad = math.radians(self.yaw)
        return (math.cos(rad), math.sin(rad))


class SearchPattern(Enum):
    """Search patterns for when target is lost."""
    HOVER = "hover"           
    SPIRAL = "spiral"         # Expanding spiral from last known position
    RETRACE = "retrace"       # Go back along predicted path
    RETURN_HOME = "return"    # Return to starting position


class CoordinationPlatform:
    """
    Central coordination hub implementing the leader-follower problem.
    
    This class:
    1. Receives position updates from UGV
    2. Calculates optimal follow waypoints for UAV
    3. Manages system state machine
    4. Handles target lost/reacquired logic
    5. Publishes waypoints to UAV
    
    Usage:
        broker = MessageBroker()
        coord = CoordinationPlatform(broker)
        coord.set_parameters(follow_distance=15.0, altitude=30.0)
        coord.start()
        
        In the main loop:
        coord.update()
    """
    
    def __init__(self, broker: MessageBroker, config: Dict = None):
        """
        Initialize the Coordination Platform.
        
        Args:
            broker: MessageBroker for communication
            config: Configuration dictionary
        """
        self.broker = broker
        self.config = config or COORDINATION_CONFIG
        
        # Follow parameters
        self.follow_distance = UAV_CONFIG['default_follow_distance']
        self.follow_altitude = UAV_CONFIG['default_altitude']
        self.lateral_offset = 0.0
        
        # State tracking
        self.current_state = SystemState.IDLE
        self.previous_state = SystemState.IDLE
        
        # Vehicle states
        self.ugv_state = VehicleState()
        self.uav_state = VehicleState()
        
        # Target lost handling
        self.lost_start_time: Optional[float] = None
        self.last_known_ugv_state: Optional[VehicleState] = None
        self.search_pattern = SearchPattern.HOVER
        self.search_time = 0.0
        
        # Tracking statistics
        self.stats = {
            'waypoints_generated': 0,
            'time_tracking': 0.0,
            'time_lost': 0.0,
            'times_lost': 0,
            'times_reacquired': 0,
        }
        
        # Timing
        self.start_time: Optional[float] = None
        self.last_update_time = time.time()
        self.last_waypoint_time = 0.0
        
        # Final destination
        self.final_destination: Optional[Tuple[float, float, float]] = None
        self.destination_reached = False
        
        # Callbacks for state change notifications
        self._state_change_callbacks: list[Callable] = []
        
        # Subscribe to vehicle position updates
        self.broker.subscribe(TOPICS['UGV_POSITION'], self._on_ugv_position)
        self.broker.subscribe(TOPICS['UAV_POSITION'], self._on_uav_position)
        
        print("[Coordination] Platform initialized")
    
    def set_parameters(self, 
                       follow_distance: Optional[float] = None,
                       altitude: Optional[float] = None,
                       lateral_offset: Optional[float] = None) -> None:
        """
        Set follow parameters.
        
        Args:
            follow_distance: How far behind the UGV to follow (meters)
            altitude: How high above ground to fly (meters)
            lateral_offset: Offset to the side (positive = right of UGV)
        """
        if follow_distance is not None:
            self.follow_distance = follow_distance
        if altitude is not None:
            self.follow_altitude = altitude
        if lateral_offset is not None:
            self.lateral_offset = lateral_offset
        
        print(f"[Coordination] Parameters updated: "
              f"distance={self.follow_distance}m, "
              f"altitude={self.follow_altitude}m, "
              f"lateral={self.lateral_offset}m")
    
    def set_final_destination(self, x: float, y: float, z: float) -> None:
        """
        Set the UGV's final destination for mission completion detection.
        
        Args:
            x, y, z: Destination coordinates
        """
        self.final_destination = (x, y, z)
        print(f"[Coordination] Final destination set: ({x:.1f}, {y:.1f}, {z:.1f})")
    
    def start(self) -> None:
        """
        Start the coordination platform.
        """
        self._transition_to_state(SystemState.INITIALIZING)
        self.start_time = time.time()
        print("[Coordination] Platform started - entering INITIALIZING state")
    
    def stop(self) -> None:
        """Stop the coordination platform."""
        self._transition_to_state(SystemState.IDLE)
        self._publish_status("Coordination platform stopped")
        print("[Coordination] Platform stopped")
    
    def update(self) -> SystemState:
        """
        Main update loop
        
        This method:
        1. Checks for timeout conditions
        2. Updates state machine
        3. Calculates and publishes new waypoints
        
        Returns:
            Current system state
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        
        if self.current_state == SystemState.INITIALIZING:
            self._handle_initializing_state()
            
        elif self.current_state == SystemState.TRACKING:
            self._handle_tracking_state(dt)
            
        elif self.current_state == SystemState.TARGET_LOST:
            self._handle_target_lost_state(dt)
            
        elif self.current_state == SystemState.SIMULATION_COMPLETE:
            pass  # Nothing to do
            
        elif self.current_state == SystemState.ERROR:
            pass  # Wait for operator intervention
        
        return self.current_state
    
    def _handle_initializing_state(self) -> None:
        """Handle INITIALIZING state logic."""
        # Check if we've received positions from both vehicles
        ugv_ready = self.ugv_state.timestamp > 0
        uav_ready = self.uav_state.timestamp > 0
        
        if ugv_ready and uav_ready:
            print("[Coordination] Both vehicles ready - entering TRACKING state")
            self._transition_to_state(SystemState.TRACKING)
        elif ugv_ready:
            # If only UGV is ready, we can still start
            # (UAV might be waiting for first waypoint)
            print("[Coordination] UGV ready, starting tracking")
            self._transition_to_state(SystemState.TRACKING)
    
    def _handle_tracking_state(self, dt: float) -> None:
        """
        Handle TRACKING state logic.
        1. Check if UGV position is stale (timeout)
        2. Calculate new waypoint for UAV
        3. Check for mission completion
        """
        self.stats['time_tracking'] += dt
        
        # Check for UGV timeout
        ugv_age = self.ugv_state.age()
        if ugv_age > self.config['position_timeout']:
            print(f"[Coordination] UGV position timeout ({ugv_age:.1f}s > "
                  f"{self.config['position_timeout']}s)")
            self._transition_to_target_lost()
            return
        
        if self._check_destination_reached():
            print("[Coordination] UGV reached final destination!")
            self._transition_to_state(SystemState.SIMULATION_COMPLETE)
            return
        
        self._generate_and_send_waypoint()
    
    def _handle_target_lost_state(self, dt: float) -> None:
        """
        Handle TARGET_LOST state logic.
        - If UGV position reacquired -> return to TRACKING
        - If timeout (60s) reached -> SIMULATION_COMPLETE (unsuccessful)
        """
        self.stats['time_lost'] += dt
        self.search_time += dt
        
        # Check if UGV position has been reacquired
        ugv_age = self.ugv_state.age()
        if ugv_age < self.config['position_timeout']:
            print(f"[Coordination] UGV position reacquired after "
                  f"{self.search_time:.1f}s")
            self.stats['times_reacquired'] += 1
            self._transition_to_state(SystemState.TRACKING)
            self.search_time = 0.0
            return
        
        # Check for search timeout
        time_lost = time.time() - self.lost_start_time
        if time_lost > self.config['search_timeout']:
            print(f"[Coordination] Search timeout ({time_lost:.1f}s) - "
                  "mission failed")
            self._transition_to_state(SystemState.SIMULATION_COMPLETE)
            return
        
        self._execute_search_pattern()
    
    def _transition_to_target_lost(self) -> None:
        """Transition to TARGET_LOST state with proper setup."""
        self.last_known_ugv_state = VehicleState(
            x=self.ugv_state.x,
            y=self.ugv_state.y,
            z=self.ugv_state.z,
            yaw=self.ugv_state.yaw,
            velocity=self.ugv_state.velocity,
            timestamp=self.ugv_state.timestamp
        )
        self.lost_start_time = time.time()
        self.search_time = 0.0
        self.stats['times_lost'] += 1
        
        self._transition_to_state(SystemState.TARGET_LOST)
    
    def _transition_to_state(self, new_state: SystemState) -> None:
        """
        Transition to a new state with proper notifications.
        
        Args:
            new_state: The state to transition to
        """
        if new_state == self.current_state:
            return
        
        self.previous_state = self.current_state
        self.current_state = new_state
        
        # Notify callbacks
        for callback in self._state_change_callbacks:
            try:
                callback(self.previous_state, new_state)
            except Exception as e:
                print(f"[Coordination] State callback error: {e}")

        self._publish_status(f"State changed: {self.previous_state} -> {new_state}")
        
        print(f"[Coordination] State: {self.previous_state} -> {new_state}")
    
    def _generate_and_send_waypoint(self) -> None:
        """
        Calculate optimal follow position and send waypoint to UAV.
        
        The follow position is calculated as:
        1. Start at UGV position
        2. Move backward (opposite to UGV heading) by follow_distance
        3. Add lateral offset if configured
        4. Add altitude
        
        """
        # Limit update rate
        current_time = time.time()
        if current_time - self.last_waypoint_time < 0.1:  # Max 10Hz
            return
        
        ugv = self.ugv_state
        
        # Calculate follow position
        # Get the direction the UGV is facing
        forward_x, forward_y = ugv.get_forward_vector()
        
        # Calculate position behind the UGV
        follow_x = ugv.x - forward_x * self.follow_distance
        follow_y = ugv.y - forward_y * self.follow_distance
        
        # Add lateral offset (perpendicular to heading)
        if self.lateral_offset != 0:
            # Perpendicular vector (rotate 90 degrees)
            perp_x = -forward_y
            perp_y = forward_x
            follow_x += perp_x * self.lateral_offset
            follow_y += perp_y * self.lateral_offset
        
        # Set altitude
        follow_z = ugv.z + self.follow_altitude
        
        # Optional: Predictive following
        # If UGV is moving fast, predict where it will be
        if ugv.velocity > 2.0:  # Only predict if moving significantly
            lookahead_time = self.config.get('waypoint_lookahead', 0.5)
            # Predict UGV position
            predicted_x = ugv.x + forward_x * ugv.velocity * lookahead_time
            predicted_y = ugv.y + forward_y * ugv.velocity * lookahead_time
            # Recalculate follow position based on prediction
            follow_x = predicted_x - forward_x * self.follow_distance
            follow_y = predicted_y - forward_y * self.follow_distance
            if self.lateral_offset != 0:
                follow_x += perp_x * self.lateral_offset
                follow_y += perp_y * self.lateral_offset
        
        # Calculate desired yaw (face toward UGV)
        dx = ugv.x - follow_x
        dy = ugv.y - follow_y
        desired_yaw = math.degrees(math.atan2(dy, dx))
        
        waypoint = create_waypoint_message(
            x=follow_x,
            y=follow_y,
            z=follow_z,
            yaw=desired_yaw
        )
        
        self.broker.publish(TOPICS['UAV_WAYPOINT'], waypoint, source='CoordinationPlatform')
        
        self.stats['waypoints_generated'] += 1
        self.last_waypoint_time = current_time
    
    def _execute_search_pattern(self) -> None:
        """
        Execute search pattern when target is lost.
        
        Different patterns for different scenarios:
        - HOVER: Stay at last position (simple)
        - SPIRAL: Expanding spiral from last known position
        """
        if self.last_known_ugv_state is None:
            return
        
        last = self.last_known_ugv_state
        
        if self.search_pattern == SearchPattern.HOVER:
            # Just hover at last known follow position
            forward_x, forward_y = last.get_forward_vector()
            hover_x = last.x - forward_x * self.follow_distance
            hover_y = last.y - forward_y * self.follow_distance
            hover_z = last.z + self.follow_altitude
            
            waypoint = create_waypoint_message(hover_x, hover_y, hover_z)
            self.broker.publish(TOPICS['UAV_WAYPOINT'], waypoint, 
                              source='CoordinationPlatform')
            
        elif self.search_pattern == SearchPattern.SPIRAL:
            # Expanding spiral centered on last known position
            # Spiral grows with time
            radius = 5.0 + self.search_time * 2.0  # Expand 2m/s
            angle = self.search_time * 1.0  # Rotate 1 rad/s
            
            spiral_x = last.x + radius * math.cos(angle)
            spiral_y = last.y + radius * math.sin(angle)
            spiral_z = last.z + self.follow_altitude
            
            waypoint = create_waypoint_message(spiral_x, spiral_y, spiral_z)
            self.broker.publish(TOPICS['UAV_WAYPOINT'], waypoint,
                              source='CoordinationPlatform')
    
    def _check_destination_reached(self) -> bool:
        """Check if UGV has reached its final destination."""
        if self.final_destination is None:
            return False
        
        dx = self.ugv_state.x - self.final_destination[0]
        dy = self.ugv_state.y - self.final_destination[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        return distance < 5.0  # Within 5 meters of destination
    
    def _on_ugv_position(self, message: Message) -> None:
        """Callback when UGV position update is received."""
        data = message.data
        self.ugv_state = VehicleState(
            x=data.get('x', 0),
            y=data.get('y', 0),
            z=data.get('z', 0),
            yaw=data.get('yaw', 0),
            velocity=data.get('velocity', 0),
            timestamp=data.get('timestamp', time.time())
        )
    
    def _on_uav_position(self, message: Message) -> None:
        """Callback when UAV position update is received."""
        data = message.data
        self.uav_state = VehicleState(
            x=data.get('x', 0),
            y=data.get('y', 0),
            z=data.get('z', 0),
            yaw=data.get('yaw', 0),
            velocity=data.get('velocity', 0),
            timestamp=data.get('timestamp', time.time())
        )
    
    def _publish_status(self, message: str) -> None:
        """Publish status update to message broker."""
        status = create_status_message(
            state=self.current_state,
            message=message,
            details={
                'ugv_age': self.ugv_state.age() if self.ugv_state.timestamp > 0 else None,
                'uav_age': self.uav_state.age() if self.uav_state.timestamp > 0 else None,
                'stats': self.stats
            }
        )
        self.broker.publish(TOPICS['SYSTEM_STATUS'], status, 
                          source='CoordinationPlatform')
    
    def register_state_callback(self, callback: Callable) -> None:
        """
        Register a callback to be notified of state changes.
        
        Callback signature: callback(old_state: SystemState, new_state: SystemState)
        
        Args:
            callback: Function to call on state changes
        """
        self._state_change_callbacks.append(callback)
    
    def get_statistics(self) -> Dict:
        """Get coordination statistics."""
        return {
            **self.stats,
            'current_state': self.current_state,
            'uptime': time.time() - self.start_time if self.start_time else 0,
            'ugv_state_age': self.ugv_state.age() if self.ugv_state.timestamp > 0 else None,
        }


# =============================================================================
# STANDALONE TEST
# =============================================================================
if __name__ == '__main__':
    """Test the Coordination Platform with simulated messages."""
    import threading
    
    print("Testing Coordination Platform")
    
    broker = MessageBroker()
    coord = CoordinationPlatform(broker)
    
    # Track received waypoints
    received_waypoints = []
    def on_waypoint(msg):
        received_waypoints.append(msg.data)
        print(f"  -> Waypoint generated: ({msg.data['x']:.1f}, {msg.data['y']:.1f}, {msg.data['z']:.1f})")
    
    broker.subscribe(TOPICS['UAV_WAYPOINT'], on_waypoint)
    
    coord.start()
    print(f"State: {coord.current_state}")
    
    # Simulate UGV position updates
    print("\nSimulating UGV movement")
    for i in range(10):
        # Publish simulated UGV position (moving forward)
        broker.publish(TOPICS['UGV_POSITION'], {
            'x': 100.0 + i * 5,
            'y': 50.0,
            'z': 0.0,
            'yaw': 0.0,  # Facing East
            'velocity': 5.0,
            'timestamp': time.time()
        }, source='SimulatedUGV')
        
        # Update coordination
        coord.update()
        time.sleep(0.2)
    
    print(f"\nGenerated {len(received_waypoints)} waypoints")
    print(f"Statistics: {coord.get_statistics()}")
    
    # Test target lost
    print("\nSimulating communication loss (waiting 4 seconds)")
    time.sleep(4)
    coord.update()
    print(f"State after timeout: {coord.current_state}")
    
    print("\nCoordination Platform test complete")
