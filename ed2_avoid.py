import math
import carla


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def compute_uav_speed_control(
    uav: carla.Vehicle,
    ugv: carla.Vehicle,
    target_loc: carla.Location,
    *,
    dt: float,
    follow_distance: float = 8.0,
    speed_scale: float = 1.1,       
    speed_bias: float = 1.5,        
    min_speed: float = 3.0,
    max_speed: float = 22.0,
    k_distance: float = 0.4,        
    k_throttle: float = 0.08,
    k_brake: float = 0.06,
):

    ugv_vel = ugv.get_velocity()
    ugv_speed = math.hypot(ugv_vel.x, ugv_vel.y)

    
    desired_speed = speed_scale * ugv_speed + speed_bias

    
    uav_loc = uav.get_transform().location
    dx = target_loc.x - uav_loc.x
    dy = target_loc.y - uav_loc.y
    horizontal_dist = math.hypot(dx, dy)

    distance_error = horizontal_dist - follow_distance
    desired_speed += k_distance * distance_error

   
    desired_speed = clamp(desired_speed, min_speed, max_speed)

    
    uav_vel = uav.get_velocity()
    current_speed = math.sqrt(
        uav_vel.x**2 + uav_vel.y**2 + uav_vel.z**2
    )

    speed_error = desired_speed - current_speed

    
    throttle = clamp(k_throttle * speed_error, 0.0, 1.0)
    brake = 0.0

    # If too fast → brake
    if speed_error < -1.0:
        brake = clamp(k_brake * (-speed_error), 0.0, 0.8)
        throttle = 0.0

    # -------- 6) Return control values --------
    return throttle, brake





import math
import numpy as np
import carla


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class FlightPathAvoider:
    def __init__(
        self,
        *,
        lookahead_m=20.0,
        front_half_angle_deg=40.0,
        wide_half_angle_deg=75.0,
        danger_dist_m=10.0,
        hard_brake_dist_m=5.0,
        steer_gain=1.3,
        max_avoid_steer=0.9,
        z_min=-3.0,
        z_max=3.0,
    ):
        self.lookahead_m = float(lookahead_m)
        self.front_half_angle = math.radians(front_half_angle_deg)
        self.wide_half_angle = math.radians(wide_half_angle_deg)
        self.danger_dist_m = float(danger_dist_m)
        self.hard_brake_dist_m = float(hard_brake_dist_m)
        self.steer_gain = float(steer_gain)
        self.max_avoid_steer = float(max_avoid_steer)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self._steer_avoid = 0.0
        self._speed_factor = 1.0
        self._brake_request = 0.0

    def attach_lidar(self, world: carla.World, parent: carla.Actor) -> carla.Sensor:
        bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        bp.set_attribute("range", "50.0")
        bp.set_attribute("rotation_frequency", "20.0")
        bp.set_attribute("channels", "32")
        bp.set_attribute("points_per_second", "120000")
        bp.set_attribute("upper_fov", "10.0")
        bp.set_attribute("lower_fov", "-30.0")
        rel_tf = carla.Transform(
            carla.Location(x=0.8, z=1.4),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
        )
        sensor = world.spawn_actor(bp, rel_tf, attach_to=parent)
        sensor.listen(self._on_lidar)
        return sensor

    def _on_lidar(self, lidar: carla.LidarMeasurement):
        pts = np.frombuffer(lidar.raw_data, dtype=np.float32).reshape(-1, 4)
        if pts.shape[0] == 0:
            self._steer_avoid, self._speed_factor, self._brake_request = 0.0, 1.0, 0.0
            return

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]

        # Range in XY plane (forward collision relevance)
        dist = np.sqrt(x * x + y * y)

        # Filter points to relevant band (ignore extreme z, too near, too far)
        m = (dist > 0.6) & (dist < self.lookahead_m) & (z > self.z_min) & (z < self.z_max)
        pts = pts[m]
        if pts.shape[0] == 0:
            self._steer_avoid, self._speed_factor, self._brake_request = 0.0, 1.0, 0.0
            return

        x = pts[:, 0]
        y = pts[:, 1]
        dist = np.sqrt(x * x + y * y)
        ang = np.arctan2(y, x)

        # Narrow "front" cone for imminent collision (brake / slow)
        front = (x > 0.0) & (np.abs(ang) < self.front_half_angle)
        # Wider cone for steering repulsion
        wide = (x > 0.0) & (np.abs(ang) < self.wide_half_angle)

        # ---- Speed factor + brake request from closest obstacle ahead ----
        speed_factor = 1.0
        brake_req = 0.0

        if np.any(front):
            dmin = float(np.min(dist[front]))
            if dmin <= self.hard_brake_dist_m:
                speed_factor = 0.0
                brake_req = 1.0
            elif dmin <= self.danger_dist_m:
                # Smooth slowdown between danger and hard brake distance
                t = (dmin - self.hard_brake_dist_m) / max(1e-3, (self.danger_dist_m - self.hard_brake_dist_m))
                t = float(np.clip(t, 0.0, 1.0))
                speed_factor = t
                brake_req = float(np.clip(1.0 - t, 0.0, 0.8))

        # ---- Avoid steering: push away from obstacle lateral centroid ----
        steer_avoid = 0.0
        if np.any(wide):
            # Weight closer points more
            w = 1.0 / np.maximum(dist[wide], 1.0)
            y_centroid = float(np.sum(y[wide] * w) / np.sum(w))

            # If obstacles are mostly left (y>0), steer right (positive steer)
            steer_avoid = -self.steer_gain * (y_centroid / 3.0)
            steer_avoid = float(np.clip(steer_avoid, -self.max_avoid_steer, self.max_avoid_steer))

        self._steer_avoid = steer_avoid
        self._speed_factor = float(np.clip(speed_factor, 0.0, 1.0))
        self._brake_request = float(np.clip(brake_req, 0.0, 1.0))

    def get_avoidance(self):
        """Return (steer_avoid, speed_factor, brake_request)."""
        return self._steer_avoid, self._speed_factor, self._brake_request