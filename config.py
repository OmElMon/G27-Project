"""
Configuration file for UAV-UGV Coordination System
===================================================
This file contains all shared constants, default parameters, and configuration
values used across the different subsystems.
"""

# =============================================================================
# CARLA CONNECTION SETTINGS
# =============================================================================
CARLA_HOST = 'localhost'
CARLA_PORT = 2000
CARLA_TIMEOUT = 10.0  # seconds

# =============================================================================
# UGV CONFIGURATION
# =============================================================================
UGV_CONFIG = {
    # Vehicle blueprint - can be changed to any vehicle in CARLA's catalog
    # See: https://carla.readthedocs.io/en/latest/catalogue_vehicles/
    'blueprint': 'vehicle.jeep.wrangler_rubicon',
    
    # Navigation settings
    'ignore_traffic_lights': True,  # For demo purposes
    'ignore_stop_signs': True,      # For demo purposes
    'follow_speed_limits': False,   # For demo purposes
    'target_speed': 30.0,           # km/h when not following speed limits
}

# =============================================================================
# UAV CONFIGURATION
# =============================================================================
UAV_CONFIG = {
    # Since CARLA doesn't have native UAV support, we use a static prop
    # and control it with disabled physics. This prop will represent our drone.
    # Options: 'static.prop.box01', 'static.prop.briefcase', etc.
    # We'll use a small object or create an invisible sensor platform
    'blueprint': None,  # None = invisible sensor platform (TODO: find a prop or upload custom prop)
    
    # Physical constraints (kinematic model)
    # These values simulate a medium-sized quadcopter
    'max_speed': 15.0,              # m/s (about 54 km/h - typical for commercial drones)
    'max_acceleration': 5.0,        # m/s^2 (how quickly it can speed up/slow down)
    'max_vertical_speed': 8.0,      # m/s (climbing/descending rate)
    'max_yaw_rate': 90.0,           # degrees/s (how fast it can rotate)
    
    # Default follow parameters
    'default_altitude': 30.0,       # meters above ground
    'default_follow_distance': 15.0, # meters behind the UGV
    
    # Control loop settings
    'position_tolerance': 2.0,      # meters - considered "at target" if within this distance
    'update_rate': 0.05,            # seconds between control updates (20 Hz)
}

# =============================================================================
# COORDINATION PLATFORM CONFIGURATION
# =============================================================================
COORDINATION_CONFIG = {
    # Communication timeout settings (from state diagram - can change these later)
    'position_timeout': 3.0,        # seconds - triggers "target lost" state
    'search_timeout': 60.0,         # seconds - max time in search mode before giving up
    
    # Waypoint generation settings
    'waypoint_lookahead': 2.0,      # seconds - predict where UGV will be
    'min_waypoint_distance': 3.0,   # meters - don't generate new waypoint if UAV is close
}

# =============================================================================
# USER INTERFACE CONFIGURATION
# =============================================================================
UI_CONFIG = {
    'spectator_distance': 20.0,     # meters behind vehicle
    'spectator_height': 15.0,       # meters above vehicle
    'spectator_pitch': -30.0,       # degrees (looking down)
}

# =============================================================================
# SYSTEM STATES
# =============================================================================
class SystemState:
    """Enumeration of system states from the state diagram."""
    IDLE = 'IDLE'
    INITIALIZING = 'INITIALIZING'
    TRACKING = 'TRACKING'
    TARGET_LOST = 'TARGET_LOST'
    ERROR = 'ERROR'
    SIMULATION_COMPLETE = 'SIMULATION_COMPLETE'
