# uav_follow_ugv.py
# Control a "UAV" (CARLA vehicle actor used as drone) to follow a UGV in CARLA:
# - Throttle/steer to reduce horizontal distance
# - PID altitude controller commanding vertical velocity via set_target_velocity()

import math
import time
import carla


# ----------------------------
# Small helpers
# ----------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ----------------------------
# PID Controller
# ----------------------------
class PID:
    def __init__(self, kp, ki, kd, i_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)
        self.i = 0.0
        self.prev_e = None

    def reset(self):
        self.i = 0.0
        self.prev_e = None

    def step(self, e, dt):
        if dt <= 0:
            return 0.0

        self.i += e * dt
        self.i = clamp(self.i, -self.i_limit, self.i_limit)

        if self.prev_e is None:
            de = 0.0
        else:
            de = (e - self.prev_e) / dt

        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * de


# ----------------------------
# Find actors (by role_name or id)
# ----------------------------
def find_actor_by_role(world, role_name: str):
    for a in world.get_actors():
        if a.attributes.get("role_name") == role_name:
            return a
    return None

def find_actor_by_id(world, actor_id: int):
    return world.get_actor(actor_id)


# ----------------------------
# Main follower controller
# ----------------------------
def run_follow_controller(
    host="127.0.0.1",
    port=2000,
    ugv_role="ugv",
    uav_role="uav",
    ugv_id=None,
    uav_id=None,
    altitude_above_ugv=15.0,     # meters
    follow_distance=8.0,         # meters behind ugv (horizontal plane)
    max_uav_speed=18.0,          # m/s (~40 mph)
    min_uav_speed=0.0,
):
    client = carla.Client(host, port)
    client.set_timeout(5.0)

    world = client.get_world()
    settings = world.get_settings()

    # Enable synchronous mode for stable control (recommended)
    original_settings = settings
    if not settings.synchronous_mode:
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz
        world.apply_settings(settings)

    try:
        # Resolve actors
        if ugv_id is not None:
            ugv = find_actor_by_id(world, int(ugv_id))
        else:
            ugv = find_actor_by_role(world, ugv_role)

        if uav_id is not None:
            uav = find_actor_by_id(world, int(uav_id))
        else:
            uav = find_actor_by_role(world, uav_role)

        if ugv is None:
            raise RuntimeError(f"Could not find UGV actor (role_name='{ugv_role}' or id={ugv_id}).")
        if uav is None:
            raise RuntimeError(f"Could not find UAV actor (role_name='{uav_role}' or id={uav_id}).")

        if not isinstance(uav, carla.Vehicle):
            raise RuntimeError("UAV actor must be a carla.Vehicle for this example (apply_control + set_target_velocity).")

        # PID controllers
        # Altitude: output is desired vertical speed (m/s)
        alt_pid = PID(kp=1.2, ki=0.0, kd=0.35, i_limit=5.0)

        # Speed: output used to set throttle (0..1)
        spd_pid = PID(kp=0.08, ki=0.02, kd=0.01, i_limit=2.0)

        # Steering: proportional on heading error (simple and effective)
        k_steer = 0.9

        prev_time = time.time()

        print("Starting UAV follow controller...")
        print(f"UGV id={ugv.id}, UAV id={uav.id}")
        print("Press Ctrl+C to stop.\n")

        while True:
            world.tick()
            now = time.time()
            dt = now - prev_time
            prev_time = now

            ugv_tf = ugv.get_transform()
            ugv_loc = ugv_tf.location
            ugv_yaw = math.radians(ugv_tf.rotation.yaw)

            uav_tf = uav.get_transform()
            uav_loc = uav_tf.location
            uav_yaw = math.radians(uav_tf.rotation.yaw)

            # ----------------------------
            # Compute desired follow point
            # ----------------------------
            # Follow behind the UGV in the horizontal plane by follow_distance,
            # and keep altitude_above_ugv meters above its current Z.
            behind_vec = carla.Vector3D(
                x=-math.cos(ugv_yaw) * follow_distance,
                y=-math.sin(ugv_yaw) * follow_distance,
                z=0.0
            )
            target_loc = carla.Location(
                x=ugv_loc.x + behind_vec.x,
                y=ugv_loc.y + behind_vec.y,
                z=ugv_loc.z + altitude_above_ugv
            )

            # Horizontal vector to target
            dx = target_loc.x - uav_loc.x
            dy = target_loc.y - uav_loc.y
            horiz_dist = math.hypot(dx, dy)

            # Desired heading to target
            desired_yaw = math.atan2(dy, dx)
            yaw_err = wrap_to_pi(desired_yaw - uav_yaw)

            # ----------------------------
            # STEER: turn toward the target
            # ----------------------------
            steer_cmd = clamp(k_steer * yaw_err, -1.0, 1.0)

            # ----------------------------
            # THROTTLE: control UAV forward speed based on distance
            # ----------------------------
            # Simple distance->desired speed mapping
            # (farther away => faster, close => slower)
            desired_speed = clamp(horiz_dist * 0.8, min_uav_speed, max_uav_speed)

            v = uav.get_velocity()
            current_speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

            speed_err = desired_speed - current_speed
            throttle_cmd = spd_pid.step(speed_err, dt)
            throttle_cmd = clamp(throttle_cmd, 0.0, 1.0)

            # Optional: apply brake if overshooting speed a lot
            brake_cmd = 0.0
            if speed_err < -2.0:
                brake_cmd = clamp((-speed_err) * 0.05, 0.0, 0.6)
                throttle_cmd = 0.0

            # ----------------------------
            # ALTITUDE: command vertical velocity
            # ----------------------------
            alt_err = target_loc.z - uav_loc.z
            vz_cmd = alt_pid.step(alt_err, dt)
            vz_cmd = clamp(vz_cmd, -6.0, 6.0)  # limit climb rate

            # Keep current XY velocities, override Z velocity toward our vz_cmd
            # This is a pragmatic method for altitude control in CARLA.
            new_vel = carla.Vector3D(v.x, v.y, vz_cmd)
            uav.set_target_velocity(new_vel)

            # Apply vehicle control for horizontal chase
            ctrl = carla.VehicleControl()
            ctrl.throttle = float(throttle_cmd)
            ctrl.steer = float(steer_cmd)
            ctrl.brake = float(brake_cmd)
            ctrl.hand_brake = False
            ctrl.reverse = False
            uav.apply_control(ctrl)

            # Debug print (light)
            # print(f"dist={horiz_dist:5.1f}m  alt_err={alt_err:5.1f}m  spd={current_speed:4.1f}  thr={throttle_cmd:.2f}  steer={steer_cmd:.2f}  vz={vz_cmd:.2f}")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        # restore settings
        world.apply_settings(original_settings)
        print("World settings restored.")


if __name__ == "__main__":
    # Use either role_name matching or hard-coded ids.
    # Example:
    #   run_follow_controller(ugv_role="hero", uav_role="uav")
    # or:
    #   run_follow_controller(ugv_id=123, uav_id=456)

    run_follow_controller(
        ugv_role="ugv",
        uav_role="uav",
        ugv_id=None,
        uav_id=None,
        altitude_above_ugv=15.0,
        follow_distance=8.0,
        max_uav_speed=18.0,
    )
