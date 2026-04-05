"""
Configuration file for UAV-UGV Coordination System
===================================================
This file contains all shared constants, default parameters, and configuration
values used across the different subsystems.

Owner: Sean Bowden
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

    # LIDAR sensor settings
    'lidar_channels': 32,               # Number of vertical laser lines
    'lidar_range': 50.0,                # Max detection range (meters)
    'lidar_points_per_second': 300000,  # Total points generated per second
    'lidar_rotation_frequency': 20.0,   # Full rotations per second (Hz)
    'lidar_upper_fov': 10.0,            # Upper vertical FOV limit (degrees)
    'lidar_lower_fov': -30.0,           # Lower vertical FOV limit (degrees)

    # Obstacle detection settings
    'obstacle_distance_threshold': 15.0,  # meters - obstacles closer than this are reported
    'obstacle_min_points': 5,             # minimum point count to consider a cluster an obstacle
    'obstacle_ground_threshold': 0.3,     # meters - points below this height are ground, not obstacles
}

# =============================================================================
# UAV CONFIGURATION
# =============================================================================
UAV_CONFIG = {
    # Since CARLA doesn't have native UAV support, we use a static prop
    # and control it with disabled physics. This prop will represent our drone.
    'blueprint': None,  # None = invisible sensor platform (just sensors, no mesh)
    
    # Physical constraints (kinematic model)
    # These values simulate a medium-sized quadcopter
    'max_speed': 15.0,              # m/s
    'max_acceleration': 5.0,        # m/s^2
    'max_vertical_speed': 8.0,      # m/s
    'max_yaw_rate': 90.0,           # degrees/s (how fast it can rotate)
    
    # Default follow parameters
    'default_altitude': 30.0,
    'default_follow_distance': 25.0,
    
    # Control loop settings
    'position_tolerance': 2.0,      # meters - considered "at target" if within this distance
    'update_rate': 0.05,            # seconds between control updates (20 Hz)
}

# =============================================================================
# COORDINATION PLATFORM CONFIGURATION
# =============================================================================
COORDINATION_CONFIG = {
    # Communication timeout settings
    'position_timeout': 3.0,        # seconds - triggers "target lost" state
    'search_timeout': 60.0,         # seconds - max time in search mode before giving up
    
    # Waypoint generation settings
    'waypoint_lookahead': 2.0,      # seconds - predict where UGV will be
    'min_waypoint_distance': 3.0,   # meters - don't generate new waypoint if UAV is close
}

# =============================================================================
# MESSAGE BROKER CONFIGURATION
# =============================================================================
# Topic names for the pub/sub system
# naming convention: subsystem/data_type
TOPICS = {
    # UGV publishes its position, status, and detected obstacles
    'UGV_POSITION': 'ugv/position',
    'UGV_STATUS': 'ugv/status',
    'UGV_OBSTACLES': 'ugv/obstacles',
    
    # UAV publishes its position and status
    'UAV_POSITION': 'uav/position',
    'UAV_STATUS': 'uav/status',
    
    # Coordination platform publishes waypoints for UAV
    'UAV_WAYPOINT': 'coordination/uav_waypoint',
    
    # System control messages
    'SYSTEM_COMMAND': 'system/command',
    'SYSTEM_STATUS': 'system/status',
}

# =============================================================================
# DATA LOGGER CONFIGURATION
# =============================================================================
LOGGER_CONFIG = {
    'log_directory': './logs',
    'log_interval': 0.5,            # seconds between log entries
    'log_to_console': True,         # print logs to terminal
    'log_to_file': True,            # save logs to CSV
}

# =============================================================================
# USER INTERFACE CONFIGURATION
# =============================================================================
UI_CONFIG = {
    'spectator_distance': 20.0,     # meters behind vehicle
    'spectator_height': 15.0,       # meters above vehicle
    'spectator_pitch': -30.0,       # degrees
    'spectator_smoothing': 0.08,    # lerp factor per tick (0.0=frozen, 1.0=instant snap)
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
