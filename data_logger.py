"""
Data Logger for UAV-UGV Coordination System
============================================
Subscribes to message broker topics and records simulation data to
CSV files and/or console output for post-simulation analysis.

Each simulation run creates a timestamped subdirectory under the
configured log directory. Separate CSV files are written for each
data type (positions, obstacles, state changes) so they can be
loaded and plotted independently.

Owner: Sean Bowden
"""

import os
import csv
import time
import math
from datetime import datetime
from typing import Optional, Dict, Any, List

from config import LOGGER_CONFIG, TOPICS
from message_broker import MessageBroker, Message


class DataLogger:
    """
    Logs simulation data from the message broker to CSV files and console.

    Subscribes to:
        - ugv/position  -> positions.csv
        - uav/position  -> positions.csv
        - ugv/obstacles -> obstacles.csv
        - system/status -> state_changes.csv

    Also computes derived metrics each log interval:
        - UAV-UGV horizontal separation
        - UAV-UGV total (3D) separation

    Usage:
        broker = MessageBroker()
        logger = DataLogger(broker)
        logger.start()

        In main loop:
        logger.update()

        When done:
        logger.stop()
    """

    def __init__(self, broker: MessageBroker, config: Dict = None):
        """
        Initialize the data logger.

        Args:
            broker: MessageBroker to subscribe to
            config: Logger configuration (uses LOGGER_CONFIG defaults if None)
        """
        self.broker = broker
        self.config = config or LOGGER_CONFIG

        # Settings
        self.log_interval = self.config.get('log_interval', 0.5)
        self.log_to_console = self.config.get('log_to_console', True)
        self.log_to_file = self.config.get('log_to_file', True)

        # State
        self.is_running = False
        self.start_time: Optional[float] = None
        self.last_log_time = 0.0
        self.log_dir: Optional[str] = None

        # CSV writers (initialized in start())
        self._position_file = None
        self._position_writer = None
        self._obstacle_file = None
        self._obstacle_writer = None
        self._state_file = None
        self._state_writer = None

        # Cached latest data from broker callbacks
        self._ugv_pos: Optional[Dict] = None
        self._uav_pos: Optional[Dict] = None
        self._obstacles: Optional[Dict] = None

        # Subscribe to topics
        self.broker.subscribe(TOPICS['UGV_POSITION'], self._on_ugv_position)
        self.broker.subscribe(TOPICS['UAV_POSITION'], self._on_uav_position)
        self.broker.subscribe(TOPICS['UGV_OBSTACLES'], self._on_obstacles)
        self.broker.subscribe(TOPICS['SYSTEM_STATUS'], self._on_state_change)

    def start(self) -> str:
        """
        Start logging. Creates the output directory and CSV files.

        Returns:
            Path to the log directory for this run
        """
        self.start_time = time.time()
        self.is_running = True

        if self.log_to_file:
            self._create_log_directory()
            self._open_csv_files()

        print(f"[Logger] Started — writing to {self.log_dir}")
        return self.log_dir

    def stop(self) -> None:
        """Stop logging and close all files."""
        self.is_running = False
        self._close_csv_files()
        print(f"[Logger] Stopped — logs saved to {self.log_dir}")

    def update(self) -> None:
        """
        Called each simulation tick. Writes a log entry if enough
        time has elapsed since the last one.
        """
        if not self.is_running:
            return

        current_time = time.time()
        if current_time - self.last_log_time < self.log_interval:
            return

        self.last_log_time = current_time
        elapsed = current_time - self.start_time

        self._log_positions(elapsed)
        self._log_obstacles(elapsed)

    # topic callbacks

    def _on_ugv_position(self, message: Message) -> None:
        self._ugv_pos = message.data

    def _on_uav_position(self, message: Message) -> None:
        self._uav_pos = message.data

    def _on_obstacles(self, message: Message) -> None:
        self._obstacles = message.data

    def _on_state_change(self, message: Message) -> None:
        """Log state changes immediately (not throttled)."""
        data = message.data
        elapsed = time.time() - self.start_time if self.start_time else 0.0

        if self.log_to_console:
            print(f"[Logger] [{elapsed:7.1f}s] STATE: {data.get('state', '?')} "
                  f"- {data.get('message', '')}")

        if self._state_writer:
            self._state_writer.writerow([
                f"{elapsed:.2f}",
                data.get('state', ''),
                data.get('message', ''),
            ])
            self._state_file.flush()

    # periodic logging

    def _log_positions(self, elapsed: float) -> None:
        """Log current UGV/UAV positions and separation."""
        ugv = self._ugv_pos
        uav = self._uav_pos

        if ugv is None:
            return

        ugv_x, ugv_y, ugv_z = ugv.get('x', 0), ugv.get('y', 0), ugv.get('z', 0)
        ugv_yaw = ugv.get('yaw', 0)
        ugv_vel = ugv.get('velocity', 0)

        uav_x = uav.get('x', 0) if uav else 0
        uav_y = uav.get('y', 0) if uav else 0
        uav_z = uav.get('z', 0) if uav else 0
        uav_vel = uav.get('velocity', 0) if uav else 0

        # Compute separation
        dx = uav_x - ugv_x
        dy = uav_y - ugv_y
        dz = uav_z - ugv_z
        horizontal_sep = math.sqrt(dx**2 + dy**2)
        total_sep = math.sqrt(dx**2 + dy**2 + dz**2)

        if self.log_to_console:
            print(f"[Logger] [{elapsed:7.1f}s] "
                  f"UGV({ugv_x:7.1f},{ugv_y:7.1f}) vel={ugv_vel:5.1f} | "
                  f"UAV({uav_x:7.1f},{uav_y:7.1f},{uav_z:7.1f}) vel={uav_vel:5.1f} | "
                  f"sep={horizontal_sep:5.1f}m h, {total_sep:5.1f}m 3D")

        if self._position_writer:
            self._position_writer.writerow([
                f"{elapsed:.2f}",
                f"{ugv_x:.2f}", f"{ugv_y:.2f}", f"{ugv_z:.2f}",
                f"{ugv_yaw:.1f}", f"{ugv_vel:.2f}",
                f"{uav_x:.2f}", f"{uav_y:.2f}", f"{uav_z:.2f}",
                f"{uav_vel:.2f}",
                f"{horizontal_sep:.2f}", f"{total_sep:.2f}",
            ])
            self._position_file.flush()

    def _log_obstacles(self, elapsed: float) -> None:
        """Log detected LIDAR obstacles."""
        if self._obstacles is None:
            return

        obstacles = self._obstacles.get('obstacles', [])
        if not obstacles:
            return

        if self.log_to_console:
            print(f"[Logger] [{elapsed:7.1f}s] "
                  f"OBSTACLES: {len(obstacles)} detected — "
                  f"closest at {min(o['distance'] for o in obstacles):.1f}m")

        if self._obstacle_writer:
            for obs in obstacles:
                centroid = obs.get('centroid', {})
                self._obstacle_writer.writerow([
                    f"{elapsed:.2f}",
                    f"{centroid.get('x', 0):.2f}",
                    f"{centroid.get('y', 0):.2f}",
                    f"{centroid.get('z', 0):.2f}",
                    f"{obs.get('distance', 0):.2f}",
                    f"{obs.get('angle', 0):.1f}",
                    str(obs.get('point_count', 0)),
                ])
            self._obstacle_file.flush()

    # file management

    def _create_log_directory(self) -> None:
        """Create a timestamped directory for this simulation run."""
        base_dir = self.config.get('log_directory', './logs')
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.log_dir = os.path.join(base_dir, timestamp)
        os.makedirs(self.log_dir, exist_ok=True)

    def _open_csv_files(self) -> None:
        """Open CSV files and write headers."""
        if self.log_dir is None:
            return

        # Positions CSV
        pos_path = os.path.join(self.log_dir, 'positions.csv')
        self._position_file = open(pos_path, 'w', newline='')
        self._position_writer = csv.writer(self._position_file)
        self._position_writer.writerow([
            'elapsed_s',
            'ugv_x', 'ugv_y', 'ugv_z', 'ugv_yaw', 'ugv_vel',
            'uav_x', 'uav_y', 'uav_z', 'uav_vel',
            'horizontal_separation', 'total_separation',
        ])

        # Obstacles CSV
        obs_path = os.path.join(self.log_dir, 'obstacles.csv')
        self._obstacle_file = open(obs_path, 'w', newline='')
        self._obstacle_writer = csv.writer(self._obstacle_file)
        self._obstacle_writer.writerow([
            'elapsed_s',
            'centroid_x', 'centroid_y', 'centroid_z',
            'distance', 'angle', 'point_count',
        ])

        # State changes CSV
        state_path = os.path.join(self.log_dir, 'state_changes.csv')
        self._state_file = open(state_path, 'w', newline='')
        self._state_writer = csv.writer(self._state_file)
        self._state_writer.writerow([
            'elapsed_s', 'state', 'message',
        ])

    def _close_csv_files(self) -> None:
        """Flush and close all open CSV files."""
        for f in [self._position_file, self._obstacle_file, self._state_file]:
            if f:
                f.flush()
                f.close()

        self._position_file = None
        self._position_writer = None
        self._obstacle_file = None
        self._obstacle_writer = None
        self._state_file = None
        self._state_writer = None
