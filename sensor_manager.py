"""
sensor_manager.py

Minimal sensor manager for attaching GNSS, IMU, and RGB camera
to CARLA vehicle actors in a clean and reusable way.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional

import carla


class SensorManager:
    """Handles sensor blueprint lookup, attachment, and cleanup."""

    def __init__(self, world: carla.World, logger: Optional[logging.Logger] = None) -> None:
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        self.log = logger or logging.getLogger(__name__)
        self.sensors: List[carla.Actor] = []

    def attach_gnss(
        self,
        parent_actor: carla.Actor,
        transform: Optional[carla.Transform] = None,
        attributes: Optional[Dict[str, str]] = None,
    ) -> carla.Actor:
        blueprint = self.blueprint_library.find("sensor.other.gnss")
        self._apply_attributes(blueprint, attributes)

        sensor_transform = transform or carla.Transform(carla.Location(x=0.0, z=2.0))
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

        sensor_transform = transform or carla.Transform(carla.Location(x=0.0, z=2.0))
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

        default_attributes = {
            "image_size_x": "800",
            "image_size_y": "600",
            "fov": "90",
        }
        if attributes:
            default_attributes.update(attributes)

        self._apply_attributes(blueprint, default_attributes)

        sensor_transform = transform or carla.Transform(
            carla.Location(x=1.5, z=2.4),
            carla.Rotation(pitch=-10.0)
        )
        sensor = self.world.spawn_actor(blueprint, sensor_transform, attach_to=parent_actor)
        self.sensors.append(sensor)

        self.log.info("RGB camera attached to actor %s", parent_actor.id)
        return sensor

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
    def _apply_attributes(blueprint: carla.ActorBlueprint, attributes: Optional[Dict[str, str]]) -> None:
        if not attributes:
            return

        for key, value in attributes.items():
            if blueprint.has_attribute(key):
                blueprint.set_attribute(key, str(value))
                
    def listen_gnss(self, gnss_sensor: carla.Actor) -> None:
        gnss_sensor.listen(lambda data: self._on_gnss_event(data))


    def listen_imu(self, imu_sensor: carla.Actor) -> None:
        imu_sensor.listen(lambda data: self._on_imu_event(data))


    def listen_rgb_camera(self, camera_sensor: carla.Actor) -> None:
        camera_sensor.listen(lambda image: self._on_rgb_event(image))


    def _on_gnss_event(self, data) -> None:
        self.log.debug(
            "GNSS lat=%s lon=%s alt=%s",
            getattr(data, "latitude", None),
            getattr(data, "longitude", None),
            getattr(data, "altitude", None),
        )


    def _on_imu_event(self, data) -> None:
        self.log.debug(
            "IMU accel=%s gyro=%s compass=%s",
            getattr(data, "accelerometer", None),
            getattr(data, "gyroscope", None),
            getattr(data, "compass", None),
        )


    def _on_rgb_event(self, image) -> None:
        self.log.debug(
            "RGB frame received: frame=%s width=%s height=%s",
            getattr(image, "frame", None),
            getattr(image, "width", None),
            getattr(image, "height", None),
        )
        
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
    
def get_default_gnss_transform(self) -> carla.Transform:
    return self.DEFAULT_GNSS_TRANSFORM

def get_default_imu_transform(self) -> carla.Transform:
    return self.DEFAULT_IMU_TRANSFORM

def get_default_rgb_transform(self) -> carla.Transform:
    return self.DEFAULT_RGB_TRANSFORM

def get_default_rgb_attributes(self) -> dict:
    return dict(self.DEFAULT_RGB_ATTRIBUTES)

def attach_default_sensor_suite(self, parent_actor: carla.Actor) -> dict:
    gnss = self.attach_gnss(
        parent_actor,
        transform=self.get_default_gnss_transform()
    )
    imu = self.attach_imu(
        parent_actor,
        transform=self.get_default_imu_transform()
    )
    camera = self.attach_rgb_camera(
        parent_actor,
        transform=self.get_default_rgb_transform(),
        attributes=self.get_default_rgb_attributes()
    )

    return {
        "gnss": gnss,
        "imu": imu,
        "rgb_camera": camera,
    }