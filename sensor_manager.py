"""
sensor_manager.py

Minimal sensor manager for attaching GNSS, IMU, and RGB camera
to CARLA vehicle actors in a clean and reusable way.
"""

from __future__ import annotations

import csv
import logging
import os
from typing import Dict, List, Optional

import carla


class SensorManager:
    """Handles sensor blueprint lookup, attachment, listening, logging, and cleanup."""

    DEFAULT_GNSS_TRANSFORM = carla.Transform(carla.Location(x=0.0, z=2.0))
    DEFAULT_IMU_TRANSFORM = carla.Transform(carla.Location(x=0.0, z=2.0))
    DEFAULT_RGB_TRANSFORM = carla.Transform(
        carla.Location(x=1.5, z=2.4),
        carla.Rotation(pitch=-10.0)
    )

    DEFAULT_RGB_ATTRIBUTES = {
        "image_size_x": "800",
        "image_size_y": "600",
        "fov": "90",
    }

    def __init__(
        self,
        world: carla.World,
        logger: Optional[logging.Logger] = None,
        log_dir: str = "logs"
    ) -> None:
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        self.log = logger or logging.getLogger(__name__)
        self.sensors: List[carla.Actor] = []

        self.latest_gnss = None
        self.latest_imu = None
        self.latest_rgb = None

        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)

        self.gnss_log_file = os.path.join(self.log_dir, "gnss_log.csv")
        self.imu_log_file = os.path.join(self.log_dir, "imu_log.csv")

        self._initialize_csv_logs()

    def _initialize_csv_logs(self) -> None:
        """Create CSV log files with headers if they do not exist."""
        if not os.path.exists(self.gnss_log_file):
            with open(self.gnss_log_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["latitude", "longitude", "altitude"])

        if not os.path.exists(self.imu_log_file):
            with open(self.imu_log_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "accel_x", "accel_y", "accel_z",
                        "gyro_x", "gyro_y", "gyro_z",
                        "compass"
                    ]
                )

    def attach_gnss(
        self,
        parent_actor: carla.Actor,
        transform: Optional[carla.Transform] = None,
        attributes: Optional[Dict[str, str]] = None,
    ) -> carla.Actor:
        blueprint = self.blueprint_library.find("sensor.other.gnss")
        self._apply_attributes(blueprint, attributes)

        sensor_transform = transform or self.DEFAULT_GNSS_TRANSFORM
        sensor = self.world.spawn_actor(blueprint, sensor_transform, attach_to=parent_actor)
        self.sensors.append(sensor)

        self.log.info("GNSS sensor attached to actor %s", parent_actor.id)
        return sensor

    def attach_imu(
        self,
        parent_actor: carla.Actor,
        transform: Optional[carla.Transform] = None,
        attributes: Optional[Dict[str, str]] = None,
    ) -> carla.Actor:
        blueprint = self.blueprint_library.find("sensor.other.imu")
        self._apply_attributes(blueprint, attributes)

        sensor_transform = transform or self.DEFAULT_IMU_TRANSFORM
        sensor = self.world.spawn_actor(blueprint, sensor_transform, attach_to=parent_actor)
        self.sensors.append(sensor)

        self.log.info("IMU sensor attached to actor %s", parent_actor.id)
        return sensor

    def attach_rgb_camera(
        self,
        parent_actor: carla.Actor,
        transform: Optional[carla.Transform] = None,
        attributes: Optional[Dict[str, str]] = None,
    ) -> carla.Actor:
        blueprint = self.blueprint_library.find("sensor.camera.rgb")

        default_attributes = dict(self.DEFAULT_RGB_ATTRIBUTES)
        if attributes:
            default_attributes.update(attributes)

        self._apply_attributes(blueprint, default_attributes)

        sensor_transform = transform or self.DEFAULT_RGB_TRANSFORM
        sensor = self.world.spawn_actor(blueprint, sensor_transform, attach_to=parent_actor)
        self.sensors.append(sensor)

        self.log.info("RGB camera attached to actor %s", parent_actor.id)
        return sensor

    def listen_gnss(self, gnss_sensor: carla.Actor) -> None:
        gnss_sensor.listen(lambda data: self._on_gnss_event(data))

    def listen_imu(self, imu_sensor: carla.Actor) -> None:
        imu_sensor.listen(lambda data: self._on_imu_event(data))

    def listen_rgb_camera(self, camera_sensor: carla.Actor) -> None:
        camera_sensor.listen(lambda image: self._on_rgb_event(image))

    def _on_gnss_event(self, data) -> None:
        self.latest_gnss = data

        with open(self.gnss_log_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([data.latitude, data.longitude, data.altitude])

        self.log.debug(
            "GNSS lat=%s lon=%s alt=%s",
            getattr(data, "latitude", None),
            getattr(data, "longitude", None),
            getattr(data, "altitude", None),
        )

    def _on_imu_event(self, data) -> None:
        self.latest_imu = data

        with open(self.imu_log_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                data.accelerometer.x,
                data.accelerometer.y,
                data.accelerometer.z,
                data.gyroscope.x,
                data.gyroscope.y,
                data.gyroscope.z,
                data.compass,
            ])

        self.log.debug(
            "IMU accel=%s gyro=%s compass=%s",
            getattr(data, "accelerometer", None),
            getattr(data, "gyroscope", None),
            getattr(data, "compass", None),
        )

    def _on_rgb_event(self, image) -> None:
        self.latest_rgb = image

        self.log.debug(
            "RGB frame received: frame=%s width=%s height=%s",
            getattr(image, "frame", None),
            getattr(image, "width", None),
            getattr(image, "height", None),
        )

    def attach_default_sensor_suite(self, parent_actor: carla.Actor) -> dict:
        """Attach GNSS, IMU, and RGB camera with default settings."""
        gnss = self.attach_gnss(
            parent_actor,
            transform=self.DEFAULT_GNSS_TRANSFORM
        )
        imu = self.attach_imu(
            parent_actor,
            transform=self.DEFAULT_IMU_TRANSFORM
        )
        camera = self.attach_rgb_camera(
            parent_actor,
            transform=self.DEFAULT_RGB_TRANSFORM,
            attributes=self.DEFAULT_RGB_ATTRIBUTES
        )

        return {
            "gnss": gnss,
            "imu": imu,
            "rgb_camera": camera,
        }

    def get_default_gnss_transform(self) -> carla.Transform:
        return self.DEFAULT_GNSS_TRANSFORM

    def get_default_imu_transform(self) -> carla.Transform:
        return self.DEFAULT_IMU_TRANSFORM

    def get_default_rgb_transform(self) -> carla.Transform:
        return self.DEFAULT_RGB_TRANSFORM

    def get_default_rgb_attributes(self) -> dict:
        return dict(self.DEFAULT_RGB_ATTRIBUTES)

    def get_latest_gnss(self):
        return self.latest_gnss

    def get_latest_imu(self):
        return self.latest_imu

    def get_latest_rgb(self):
        return self.latest_rgb

    def destroy_all(self) -> None:
        """Safely destroy all attached sensors."""
        for sensor in self.sensors:
            try:
                if sensor.is_alive:
                    sensor.destroy()
            except Exception as e:
                self.log.warning("Failed to destroy sensor: %s", e)

        self.sensors.clear()
        self.log.info("All managed sensors destroyed.")

    @staticmethod
    def _apply_attributes(
        blueprint: carla.ActorBlueprint,
        attributes: Optional[Dict[str, str]]
    ) -> None:
        if not attributes:
            return

        for key, value in attributes.items():
            if blueprint.has_attribute(key):
                blueprint.set_attribute(key, str(value))
                
    def check_sensor_health(self):
        issues = []

        if self.latest_gnss is None:
            issues.append("GNSS not receiving data")

        if self.latest_imu is None:
            issues.append("IMU not receiving data")

        if self.latest_rgb is None:
            issues.append("RGB camera not receiving data")

        if issues:
            print(" Sensor Issues Detected:")
            for issue in issues:
                print("-", issue)
        else:
            print(" All sensors operational")