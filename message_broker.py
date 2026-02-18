"""
Message Broker for UAV-UGV Coordination System
===============================================
This module implements a simple publish-subscribe (pub/sub) message broker
that allows subsystems to communicate without direct dependencies on each other.

HOW IT WORKS:
-------------
1. Subscribers register callbacks for topics they care about
2. Publishers send messages to topics
3. Broker delivers messages to all subscribers of that topic

Example:
    broker = MessageBroker()
    
    # Coordination Platform subscribes to UGV position
    broker.subscribe('ugv/position', coordination.on_ugv_position)
    
    # UGV publishes its position
    broker.publish('ugv/position', {'x': 100, 'y': 50, 'z': 0, 'timestamp': 1234})
    
    # Coordination Platform's callback is automatically called

Owner: Sean Bowden
"""

import time
import threading
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Any, Optional
from queue import Queue, Empty
import json


@dataclass
class Message:
    """
    Represents a message in the system.
    
    Attributes:
        topic: The topic/channel this message belongs to
        data: The actual payload (dictionary of values)
        timestamp: When the message was created (for timeout detection)
        source: Which subsystem sent this message (for debugging/logging)
    """
    topic: str
    data: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)
    source: str = 'unknown'
    
    def to_dict(self) -> Dict:
        """Convert message to dictionary for logging."""
        return {
            'topic': self.topic,
            'data': self.data,
            'timestamp': self.timestamp,
            'source': self.source
        }
    
    def age(self) -> float:
        """Return how old this message is in seconds."""
        return time.time() - self.timestamp


class MessageBroker:
    """
    Central message broker implementing the pub/sub pattern.
    
    The broker keeps the last message for each topic. This allows:
    - New subscribers to immediately get the current state
    - Checking for timeouts
    - Debugging and logging
    """
    
    def __init__(self):
        """Initialize the message broker."""
        # Dictionary mapping topics to lists of callback functions
        # Using defaultdict so we don't need to check if topic exists
        self._subscribers: Dict[str, List[Callable]] = defaultdict(list)
        
        # Store the last message for each topic
        self._last_messages: Dict[str, Message] = {}
        
        # Lock for thread-safe operations
        self._lock = threading.Lock()
        
        # Message history for logging (optional)
        self._history: List[Message] = []
        self._history_enabled = True
        self._max_history = 1000
        
        # Statistics for debugging
        self._stats = {
            'messages_published': 0,
            'messages_delivered': 0,
            'topics_active': set()
        }
    
    def subscribe(self, topic: str, callback: Callable[[Message], None]) -> None:
        """
        Subscribe to a topic with a callback function.
        
        The callback will be called whenever a new message is published
        to this topic. The callback receives a Message object.
        
        Args:
            topic: The topic to subscribe to (e.g., 'ugv/position')
            callback: Function to call when message arrives
            
        Example:
            def handle_ugv_position(msg: Message):
                print(f"UGV is at {msg.data['x']}, {msg.data['y']}")
            
            broker.subscribe('ugv/position', handle_ugv_position)
        """
        with self._lock:
            self._subscribers[topic].append(callback)
            self._stats['topics_active'].add(topic)
            
    def unsubscribe(self, topic: str, callback: Callable) -> bool:
        """
        Remove a callback from a topic.
        
        Args:
            topic: The topic to unsubscribe from
            callback: The callback function to remove
            
        Returns:
            True if callback was found and removed, False otherwise
        """
        with self._lock:
            if topic in self._subscribers and callback in self._subscribers[topic]:
                self._subscribers[topic].remove(callback)
                return True
            return False
    
    def publish(self, topic: str, data: Dict[str, Any], source: str = 'unknown') -> None:
        """
        Publish a message to a topic.
        
        All subscribers to this topic will have their callbacks invoked
        with the message data.
        
        Args:
            topic: The topic to publish to (e.g., 'ugv/position')
            data: Dictionary containing the message payload
            source: Identifier of the publishing subsystem (for debugging)
            
        Example:
            broker.publish(
                'ugv/position',
                {'x': 100.5, 'y': 50.2, 'z': 0.0, 'yaw': 45.0},
                source='UGVSubsystem'
            )
        """
        # Create the message object
        message = Message(
            topic=topic,
            data=data,
            timestamp=time.time(),
            source=source
        )
        
        with self._lock:
            # Store as last message for this topic
            self._last_messages[topic] = message
            
            # Add to history if enabled
            if self._history_enabled:
                self._history.append(message)
                if len(self._history) > self._max_history:
                    self._history = self._history[-self._max_history:]
            
            self._stats['messages_published'] += 1
            
            # Get list of subscribers (copy to avoid issues during iteration)
            subscribers = list(self._subscribers.get(topic, []))
        
        # Deliver to all subscribers (outside lock to prevent deadlocks)
        for callback in subscribers:
            try:
                callback(message)
                self._stats['messages_delivered'] += 1
            except Exception as e:
                print(f"[MessageBroker] Error delivering to subscriber: {e}")
    
    def get_last_message(self, topic: str) -> Optional[Message]:
        """
        Get the most recent message for a topic.
        
        Args:
            topic: The topic to query
            
        Returns:
            The last Message object, or None if no messages yet
        """
        with self._lock:
            return self._last_messages.get(topic)
    
    def get_message_age(self, topic: str) -> Optional[float]:
        """
        Get how old the last message on a topic is.
        
        Args:
            topic: The topic to check
            
        Returns:
            Age in seconds, or None if no messages received yet
        """
        with self._lock:
            msg = self._last_messages.get(topic)
            if msg:
                return msg.age()
            return None
    
    def get_statistics(self) -> Dict:
        """Get broker statistics for debugging/monitoring."""
        with self._lock:
            return {
                'messages_published': self._stats['messages_published'],
                'messages_delivered': self._stats['messages_delivered'],
                'active_topics': list(self._stats['topics_active']),
                'subscriber_counts': {
                    topic: len(subs) for topic, subs in self._subscribers.items()
                }
            }
    
    def get_history(self, topic: Optional[str] = None, limit: int = 100) -> List[Message]:
        """
        Get message history, optionally filtered by topic.
        
        Args:
            topic: If provided, only return messages for this topic
            limit: Maximum number of messages to return
            
        Returns:
            List of Message objects, most recent last
        """
        with self._lock:
            if topic:
                filtered = [m for m in self._history if m.topic == topic]
            else:
                filtered = self._history.copy()
            return filtered[-limit:]
    
    def clear(self) -> None:
        """Clear all subscriptions, messages, and history."""
        with self._lock:
            self._subscribers.clear()
            self._last_messages.clear()
            self._history.clear()
            self._stats = {
                'messages_published': 0,
                'messages_delivered': 0,
                'topics_active': set()
            }


# =============================================================================
# POSITION MESSAGE HELPER
# =============================================================================

def create_position_message(x: float, y: float, z: float, 
                            yaw: float = 0.0, 
                            velocity: float = 0.0) -> Dict[str, Any]:
    """
    Create a standardized position message.
    
    Args:
        x, y, z: Position coordinates in CARLA world space (meters)
        yaw: Heading angle in degrees (0 = East, 90 = North in CARLA)
        velocity: Current speed in m/s
        
    Returns:
        Dictionary ready to be published
    """
    return {
        'x': x,
        'y': y,
        'z': z,
        'yaw': yaw,
        'velocity': velocity,
        'timestamp': time.time()
    }


def create_waypoint_message(x: float, y: float, z: float,
                            yaw: Optional[float] = None,
                            speed: Optional[float] = None) -> Dict[str, Any]:
    """
    Create a standardized waypoint/target message.
    
    Args:
        x, y, z: Target position coordinates
        yaw: Desired heading at waypoint (None = face direction of travel)
        speed: Desired speed when reaching waypoint (None = use default)
        
    Returns:
        Dictionary ready to be published
    """
    return {
        'x': x,
        'y': y,
        'z': z,
        'yaw': yaw,
        'speed': speed,
        'timestamp': time.time()
    }


def create_status_message(state: str, message: str = '', 
                          details: Optional[Dict] = None) -> Dict[str, Any]:
    """
    Create a standardized status message.
    
    Args:
        state: Current state (from SystemState enum)
        message: Human-readable status message
        details: Optional dictionary of additional details
        
    Returns:
        Dictionary ready to be published
    """
    return {
        'state': state,
        'message': message,
        'details': details or {},
        'timestamp': time.time()
    }


# =============================================================================
# EXAMPLE USAGE (for testing)
# =============================================================================
if __name__ == '__main__':
    print("MessageBroker Test")
    
    broker = MessageBroker()
    received_messages = []
    
    # Create a test subscriber
    def test_callback(msg: Message):
        received_messages.append(msg)
        print(f"Received: {msg.topic} -> {msg.data}")
    
    # Subscribe to a topic
    broker.subscribe('test/topic', test_callback)
    
    # Publish some messages
    broker.publish('test/topic', {'value': 1}, source='test')
    broker.publish('test/topic', {'value': 2}, source='test')
    broker.publish('other/topic', {'value': 3}, source='test')  # Won't be received
    
    # Check results
    print(f"\nReceived {len(received_messages)} messages (expected 2)")
    print(f"Last message age: {broker.get_message_age('test/topic'):.4f} seconds")
    print(f"Statistics: {broker.get_statistics()}")
    
    print("\nMessageBroker test complete")
