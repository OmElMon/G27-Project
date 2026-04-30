"""
GUIConsole — All-in-one operator console for the UAV-UGV Coordination System.

Replaces the small UAV-only CameraDisplay window in main.py with a single
larger pygame window that contains:
  - Main chase camera view (driven by a dedicated CARLA camera sensor)
  - UAV onboard camera as a picture-in-picture overlay
  - Live state readout sidebar (subscribes to SYSTEM_STATUS topic)
  - Clickable minimap that lets the operator manually re-route the UGV
  - Pause / Resume / Quit / Reset Destination buttons
  - Radio buttons to switch the main view's camera target at runtime


Owner: Sean Bowden
"""

import os
import math
import time
from typing import Optional, Tuple, List

try:
    import numpy as np
    import pygame
except ImportError as e:
    raise SystemExit(
        "GUIConsole requires pygame and numpy. Run: pip install pygame numpy"
    ) from e

import carla

from config import TOPICS, SystemState
from message_broker import MessageBroker, Message


# ---- Layout constants ------------------------------------------------------
WINDOW_W = 1280
WINDOW_H = 900

MAIN_VIEW  = pygame.Rect(0,    0,    980, 540)
UAV_PIP    = pygame.Rect(640,  350,  320, 180)   # overlaid on MAIN_VIEW
SIDEBAR    = pygame.Rect(980,  0,    300, 540)
MINIMAP    = pygame.Rect(20,   560,  1240, 280)
BOTTOM_BAR = pygame.Rect(0,    850,  1280, 50)

# Chase camera FOV (CARLA's debug overlays don't render into
# RGB cameras, so we re-create the X markers as a pygame overlay using these.)
CHASE_CAMERA_FOV = 90.0

# ---- Colors ----------------------------------------------------------------
COLOR_BG      = (18, 18, 22)
COLOR_PANEL   = (32, 32, 40)
COLOR_BORDER  = (60, 60, 70)
COLOR_TEXT    = (220, 220, 220)
COLOR_DIM     = (140, 140, 150)
COLOR_OK      = (90, 200, 110)
COLOR_WARN    = (230, 190, 60)
COLOR_ERR     = (230, 80, 80)
COLOR_UGV     = (90, 200, 255)
COLOR_UAV     = (255, 150, 70)
COLOR_DEST    = (255, 220, 90)
COLOR_PATH    = (140, 175, 220)
COLOR_BTN     = (60, 70, 90)
COLOR_BTN_HOV = (90, 110, 140)
COLOR_OBSTACLE = (220, 140, 70)    # detected obstacle dots on minimap


def _state_color(state: str) -> Tuple[int, int, int]:
    if state == SystemState.TRACKING:
        return COLOR_OK
    if state in (SystemState.TARGET_LOST, SystemState.INITIALIZING):
        return COLOR_WARN
    if state == SystemState.ERROR:
        return COLOR_ERR
    return COLOR_DIM


class _Button:
    """Minimal click-rect button. No animation, no state machine."""

    def __init__(self, label: str, rect: pygame.Rect, on_click):
        self.label = label
        self.rect = rect
        self.on_click = on_click
        self.hover = False

    def handle(self, ev) -> bool:
        if ev.type == pygame.MOUSEMOTION:
            self.hover = self.rect.collidepoint(ev.pos)
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            if self.rect.collidepoint(ev.pos):
                self.on_click()
                return True
        return False

    def draw(self, surface, font):
        color = COLOR_BTN_HOV if self.hover else COLOR_BTN
        pygame.draw.rect(surface, color, self.rect, border_radius=6)
        pygame.draw.rect(surface, COLOR_BORDER, self.rect, width=1, border_radius=6)
        text = font.render(self.label, True, COLOR_TEXT)
        surface.blit(text, text.get_rect(center=self.rect.center))


class GUIConsole:
    """All-in-one operator console window."""

    CAMERA_TARGETS = ('ugv', 'uav', 'uav_cam')
    CAMERA_LABELS = {
        'ugv':     'UGV chase',
        'uav':     'UAV chase',
        'uav_cam': 'UAV onboard',
    }

    def __init__(
        self,
        world: carla.World,
        broker: MessageBroker,
        ugv,
        uav,
        coordination,
        initial_camera_target: str = 'ugv',
    ):
        self.world = world
        self.broker = broker
        self.ugv = ugv
        self.uav = uav
        self.coordination = coordination

        self.camera_target = initial_camera_target
        # Polled by the orchestrator if it wants to actually pause subsystem
        # updates. Without that polling, the button is purely cosmetic.
        self.is_paused = False

        # Latest BGRA frame from the dedicated chase-camera sensor (set on
        # CARLA's sensor thread).
        self._latest_chase_image: Optional[carla.Image] = None

        # pygame window
        os.environ.setdefault('SDL_VIDEO_WINDOW_POS', '40,40')
        pygame.display.quit()
        pygame.init()
        self.screen = pygame.display.set_mode(
            (WINDOW_W, WINDOW_H),
            pygame.HWSURFACE | pygame.DOUBLEBUF,
        )
        pygame.display.set_caption("UAV-UGV Coordination Console")
        self.font_lg = pygame.font.SysFont('consolas', 22, bold=True)
        self.font_md = pygame.font.SysFont('consolas', 16)
        self.font_sm = pygame.font.SysFont('consolas', 14)

        # chase camera sensor
        # Free-floating camera, we set its transform manually each frame to
        # implement chase / first-person behavior. sensor_tick caps its rate
        # so we don't drown the simulator in BGRA frames.
        bp_lib = world.get_blueprint_library()
        chase_bp = bp_lib.find('sensor.camera.rgb')
        chase_bp.set_attribute('image_size_x', str(MAIN_VIEW.width))
        chase_bp.set_attribute('image_size_y', str(MAIN_VIEW.height))
        chase_bp.set_attribute('fov', str(CHASE_CAMERA_FOV))
        chase_bp.set_attribute('sensor_tick', '0.05')  # 20 Hz max
        spawn_transform = carla.Transform(
            carla.Location(x=0, y=0, z=50),
            carla.Rotation(pitch=-30, yaw=0, roll=0),
        )
        self.chase_camera: carla.Sensor = world.spawn_actor(chase_bp, spawn_transform)
        self.chase_camera.listen(self._on_chase_image)

        # Intrinsic matrix for projecting world points onto the chase image plane.
        # focal = w / (2 * tan(fov/2)) standard pinhole from horizontal FOV.
        focal = MAIN_VIEW.width / (2.0 * math.tan(math.radians(CHASE_CAMERA_FOV) / 2.0))
        self._K = np.array([
            [focal, 0.0,   MAIN_VIEW.width / 2.0],
            [0.0,   focal, MAIN_VIEW.height / 2.0],
            [0.0,   0.0,   1.0],
        ])

        # minimap world bounds
        # CARLA's coordinates are already a flat metric grid, so a linear map
        # between bounds and pixel space is sufficient. We compute bounds from
        # the union of spawn points AND road waypoints so the minimap covers
        # the whole drivable map
        carla_map = world.get_map()
        spawn_points = carla_map.get_spawn_points()
        try:
            road_waypoints = carla_map.generate_waypoints(2.0)
        except Exception as e:
            print(f"[GUIConsole] generate_waypoints failed: {e}")
            road_waypoints = []

        xs = [p.location.x for p in spawn_points]
        ys = [p.location.y for p in spawn_points]
        for wp in road_waypoints:
            xs.append(wp.transform.location.x)
            ys.append(wp.transform.location.y)
        if xs and ys:
            pad = 30.0
            self._world_min_x, self._world_max_x = min(xs) - pad, max(xs) + pad
            self._world_min_y, self._world_max_y = min(ys) - pad, max(ys) + pad
        else:
            self._world_min_x, self._world_max_x = -200, 200
            self._world_min_y, self._world_max_y = -200, 200

        # Pre-render the static road network onto a transparent surface that
        # we can blit each frame. Roads don't move during the simulation, so
        # doing the projection math once at init costs ~100ms but saves
        # thousands of per-frame transformations.
        self._road_overlay = self._build_road_overlay(road_waypoints)

        # UGV trail (drawn as a polyline on the minimap)
        self._ugv_trail: List[Tuple[float, float]] = []
        self._last_trail_record_time = 0.0

        # Latest system status string from the broker
        self._latest_status_msg = ""
        broker.subscribe(TOPICS['SYSTEM_STATUS'], self._on_system_status)

        # Latest obstacle list from the UGV's LIDAR pipeline. Centroids are in
        # SENSOR-LOCAL frame (x=fwd, y=right relative to the UGV), must be
        # rotated by UGV yaw + translated by UGV position to draw on the map.
        self._latest_obstacles: list = []
        broker.subscribe(TOPICS['UGV_OBSTACLES'], self._on_obstacles)

        # Original destination (for "Reset Dest" button)
        self._original_destination: Optional[Tuple[float, float, float]] = (
            coordination.final_destination
        )

        # ---- buttons -------------------------------------------------------
        bb = BOTTOM_BAR
        btn_y = bb.y + 10
        btn_h = bb.height - 20
        btn_w = 110
        gap = 10
        x0 = WINDOW_W - (4 * btn_w + 3 * gap) - 20
        self._buttons = [
            _Button("Reset Dest", pygame.Rect(x0 + 0 * (btn_w + gap), btn_y, btn_w, btn_h),
                    self._reset_destination),
            _Button("Pause",      pygame.Rect(x0 + 1 * (btn_w + gap), btn_y, btn_w, btn_h),
                    self._toggle_pause),
            _Button("Resume",     pygame.Rect(x0 + 2 * (btn_w + gap), btn_y, btn_w, btn_h),
                    self._resume),
            _Button("Quit",       pygame.Rect(x0 + 3 * (btn_w + gap), btn_y, btn_w, btn_h),
                    self._request_quit),
        ]

        # Camera radio rects are computed during draw, consumed during click
        self._camera_radio_rects: List[Tuple[pygame.Rect, str]] = []

        self._quit_requested = False
        self._cursor_world: Optional[Tuple[float, float]] = None

    # events
    def _on_chase_image(self, image: carla.Image) -> None:
        # Runs on CARLA's sensor thread, pygame surfaces are built lazily in draw.
        self._latest_chase_image = image

    def _on_system_status(self, msg: Message) -> None:
        data = msg.data
        if isinstance(data, dict):
            self._latest_status_msg = data.get('message', '')

    def _on_obstacles(self, msg: Message) -> None:
        data = msg.data
        if isinstance(data, dict):
            self._latest_obstacles = data.get('obstacles', []) or []

    # obstacle helpers
    def _obstacle_to_world(self, obs: dict) -> Optional[Tuple[float, float, float]]:
        """
        Rotate+translate an obstacle's sensor-local centroid into world coords.
        CARLA local frame: x=forward, y=right; rotated by UGV yaw, offset by
        UGV position.
        """
        loc = self.ugv.get_location()
        tf = self.ugv.get_transform()
        if loc is None or tf is None:
            return None
        c = obs.get('centroid', {})
        lx = float(c.get('x', 0.0))
        ly = float(c.get('y', 0.0))
        lz = float(c.get('z', 0.0))
        yaw = math.radians(tf.rotation.yaw)
        cy, sy = math.cos(yaw), math.sin(yaw)
        wx = loc.x + lx * cy - ly * sy
        wy = loc.y + lx * sy + ly * cy
        wz = loc.z + lz
        return wx, wy, wz

    # button handlers
    def _reset_destination(self) -> None:
        if self._original_destination is None:
            return
        x, y, z = self._original_destination
        self.coordination.set_final_destination(x, y, z)
        try:
            self.ugv.set_destination(carla.Location(x=x, y=y, z=z))
        except Exception:
            # Autopilot mode: coordination knows the new dest, UGV won't reroute.
            pass

    def _toggle_pause(self) -> None:
        self.is_paused = not self.is_paused

    def _resume(self) -> None:
        self.is_paused = False

    def _request_quit(self) -> None:
        self._quit_requested = True

    # road overlay
    def _build_road_overlay(self, road_waypoints: list) -> pygame.Surface:
        """
        Pre-render the static road network onto a transparent surface.

        Each drivable lane waypoint becomes a small dot on the surface; with
        2 m sampling the dots overlap or nearly touch on the minimap, giving
        the visual impression of continuous roads. Done once at init since
        the map geometry doesn't change during a run.
        """
        surf = pygame.Surface((MINIMAP.width, MINIMAP.height), pygame.SRCALPHA)

        if not road_waypoints:
            return surf

        inner = self._minimap_inner()
        # Translate inner-rect coords into surface-local coords.
        offset_x = -MINIMAP.x
        offset_y = -MINIMAP.y

        # Pre-compute scale factors for the inline projection.
        x_range = max(self._world_max_x - self._world_min_x, 1e-6)
        y_range = max(self._world_max_y - self._world_min_y, 1e-6)
        x_scale = inner.width / x_range
        y_scale = inner.height / y_range

        road_color = (110, 130, 155)   # subtle blue-gray for road surfaces

        for wp in road_waypoints:
            if wp.lane_type != carla.LaneType.Driving:
                continue
            loc = wp.transform.location
            px = int(inner.x + (loc.x - self._world_min_x) * x_scale) + offset_x
            py = int(inner.y + (loc.y - self._world_min_y) * y_scale) + offset_y
            if 0 <= px < MINIMAP.width and 0 <= py < MINIMAP.height:
                surf.set_at((px, py), road_color)

        return surf

    # minimap math
    def _minimap_inner(self) -> pygame.Rect:
        # Leave room for the title above and cursor coords below.
        r = MINIMAP.copy()
        r.x += 12
        r.y += 28
        r.width -= 24
        r.height -= 48
        return r

    def _world_to_minimap(self, x: float, y: float) -> Tuple[int, int]:
        rect = self._minimap_inner()
        u = (x - self._world_min_x) / max(self._world_max_x - self._world_min_x, 1e-6)
        v = (y - self._world_min_y) / max(self._world_max_y - self._world_min_y, 1e-6)
        return int(rect.x + u * rect.width), int(rect.y + v * rect.height)

    def _minimap_to_world(self, px: int, py: int) -> Tuple[float, float]:
        rect = self._minimap_inner()
        u = (px - rect.x) / max(rect.width, 1)
        v = (py - rect.y) / max(rect.height, 1)
        x = self._world_min_x + u * (self._world_max_x - self._world_min_x)
        y = self._world_min_y + v * (self._world_max_y - self._world_min_y)
        return x, y

    def _set_destination_from_click(self, px: int, py: int) -> None:
        x, y = self._minimap_to_world(px, py)
        z = 0.0
        ugv_loc = self.ugv.get_location()
        if ugv_loc:
            z = ugv_loc.z

        # Snap to nearest drivable lane. Without this, clicks that land off-road
        # (buildings, sidewalks, water) causing GlobalRoutePlanner.trace_route to
        # return an empty path silently
        target = carla.Location(x=x, y=y, z=z)
        try:
            wp = self.world.get_map().get_waypoint(
                target,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
            if wp is not None:
                snapped = wp.transform.location
                snap_dist = math.hypot(snapped.x - x, snapped.y - y)
                target = snapped
                if snap_dist > 5.0:
                    self._latest_status_msg = (
                        f"Snapped to nearest road ({snap_dist:.0f} m from click)"
                    )
        except Exception as e:
            self._latest_status_msg = f"Could not snap to road: {e}"

        self.coordination.set_final_destination(target.x, target.y, target.z)
        try:
            ok = self.ugv.set_destination(target)
            if not ok:
                self._latest_status_msg = (
                    "Destination updated (UGV in autopilot — not rerouting)"
                )
            elif not self.ugv.path_waypoints:
                self._latest_status_msg = (
                    "Reroute computed but path is empty (destination unreachable?)"
                )
        except Exception as e:
            self._latest_status_msg = f"Reroute failed: {e}"

    # chase cam pose
    def _update_chase_camera_transform(self) -> None:
        ugv_loc = self.ugv.get_location()
        uav_state = self.uav.get_state()
        if ugv_loc is None:
            return

        if self.camera_target == 'uav_cam':
            # First-person UAV view, looking at the UGV
            dx = ugv_loc.x - uav_state.x
            dy = ugv_loc.y - uav_state.y
            dz = ugv_loc.z - uav_state.z
            hdist = math.sqrt(dx * dx + dy * dy)
            yaw = math.degrees(math.atan2(dy, dx))
            pitch = math.degrees(math.atan2(dz, max(hdist, 1e-3)))
            transform = carla.Transform(
                carla.Location(x=uav_state.x, y=uav_state.y, z=uav_state.z - 1),
                carla.Rotation(pitch=pitch, yaw=yaw, roll=0),
            )
        elif self.camera_target == 'uav':
            # Position the camera behind+above the UAV in its heading direction,
            # then point it at the midpoint between UAV and UGV so both fit in
            # frame. Without the dynamic look-at, the camera sat at the UAV's
            # altitude (~30m) with only -15° pitch and the UGV stayed below
            # the bottom of frame.
            yaw_rad = math.radians(uav_state.yaw)
            back_dist = 22.0
            height_above = 6.0
            cx = uav_state.x - back_dist * math.cos(yaw_rad)
            cy = uav_state.y - back_dist * math.sin(yaw_rad)
            cz = uav_state.z + height_above

            target_x = (uav_state.x + ugv_loc.x) * 0.5
            target_y = (uav_state.y + ugv_loc.y) * 0.5
            target_z = (uav_state.z + ugv_loc.z) * 0.5

            dx = target_x - cx
            dy = target_y - cy
            dz = target_z - cz
            hdist = math.sqrt(dx * dx + dy * dy)
            look_yaw = math.degrees(math.atan2(dy, dx))
            look_pitch = math.degrees(math.atan2(dz, max(hdist, 1e-3)))
            transform = carla.Transform(
                carla.Location(x=cx, y=cy, z=cz),
                carla.Rotation(pitch=look_pitch, yaw=look_yaw, roll=0),
            )
        else:  # 'ugv'
            tf = self.ugv.get_transform()
            yaw = tf.rotation.yaw if tf else 0.0
            yaw_rad = math.radians(yaw)
            cx = ugv_loc.x - 18.0 * math.cos(yaw_rad)
            cy = ugv_loc.y - 18.0 * math.sin(yaw_rad)
            cz = ugv_loc.z + 12.0
            transform = carla.Transform(
                carla.Location(x=cx, y=cy, z=cz),
                carla.Rotation(pitch=-25, yaw=yaw, roll=0),
            )

        self.chase_camera.set_transform(transform)

    # ------------------------------------------------------------------ render helpers
    @staticmethod
    def _carla_image_to_surface(img: carla.Image, target_size: Tuple[int, int]):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8)
        arr = arr.reshape((img.height, img.width, 4))
        arr = arr[:, :, :3][:, :, ::-1]  # BGRA -> RGB
        surf = pygame.surfarray.make_surface(arr.swapaxes(0, 1))
        if surf.get_size() != target_size:
            surf = pygame.transform.scale(surf, target_size)
        return surf

    def _project_world_points(self, world_pts: np.ndarray,
                              w2c: np.ndarray) -> np.ndarray:
        """
        Project Nx3 CARLA world coordinates onto the chase camera image plane.

        Returns Nx2 (u, v) pixel coordinates relative to MAIN_VIEW. Rows whose
        camera-space x (forward distance) is non-positive are returned as NaN
        and should be filtered by the caller.
        """
        n = world_pts.shape[0]
        homog = np.hstack([world_pts, np.ones((n, 1))])         # Nx4
        cam = (homog @ w2c.T)[:, :3]                            # Nx3 in CARLA cam frame
        behind = cam[:, 0] <= 0.1                               # forward axis = +x

        # CARLA camera frame (x=fwd, y=right, z=up) -> standard pinhole frame
        # (x=right, y=down, z=fwd). Then project with K.
        std = np.column_stack([cam[:, 1], -cam[:, 2], cam[:, 0]])
        img = (self._K @ std.T).T
        u = img[:, 0] / img[:, 2]
        v = img[:, 1] / img[:, 2]
        out = np.column_stack([u, v])
        out[behind] = np.nan
        return out

    def _draw_path_x_overlay(self) -> None:
        """
        Draw red X markers at each UGV path waypoint, projected onto the
        chase camera image. Mirrors the in-world X markers that CARLA's
        debug.draw_string emits to the spectator viewport (which don't
        render into camera sensors).
        """
        if self._latest_chase_image is None:
            return
        try:
            wps = self.ugv.path_waypoints or []
        except Exception:
            return
        if not wps:
            return

        # Use the camera pose AT FRAME CAPTURE TIME so the marks stay
        # pixel-locked to the road instead of jittering with sensor lag.
        try:
            cam_tf = self._latest_chase_image.transform
        except AttributeError:
            cam_tf = self.chase_camera.get_transform()
        w2c = np.array(cam_tf.get_inverse_matrix())

        pts = []
        for wp_tuple in wps:
            wp = wp_tuple[0] if isinstance(wp_tuple, tuple) else wp_tuple
            loc = wp.transform.location
            pts.append([loc.x, loc.y, loc.z + 0.3])  # slight lift off road
        world_pts = np.asarray(pts, dtype=np.float64)
        image_pts = self._project_world_points(world_pts, w2c)

        x_color = (255, 50, 50)
        size = 4
        for px, py in image_pts:
            if not (np.isfinite(px) and np.isfinite(py)):
                continue
            ix = int(px) + MAIN_VIEW.x
            iy = int(py) + MAIN_VIEW.y
            if not MAIN_VIEW.collidepoint(ix, iy):
                continue
            pygame.draw.line(self.screen, x_color,
                             (ix - size, iy - size),
                             (ix + size, iy + size), 1)
            pygame.draw.line(self.screen, x_color,
                             (ix - size, iy + size),
                             (ix + size, iy - size), 1)

    def _draw_main_view(self) -> None:
        pygame.draw.rect(self.screen, (8, 8, 12), MAIN_VIEW)
        if self._latest_chase_image is not None:
            surf = self._carla_image_to_surface(
                self._latest_chase_image, (MAIN_VIEW.width, MAIN_VIEW.height))
            self.screen.blit(surf, MAIN_VIEW.topleft)
        else:
            txt = self.font_md.render(
                "Waiting for chase camera frame...", True, COLOR_DIM)
            self.screen.blit(txt, txt.get_rect(center=MAIN_VIEW.center))
        # Overlay X markers on top of the camera image, before the border.
        self._draw_path_x_overlay()
        pygame.draw.rect(self.screen, COLOR_BORDER, MAIN_VIEW, width=1)

    def _draw_uav_pip(self) -> None:
        pygame.draw.rect(self.screen, (0, 0, 0), UAV_PIP)
        img = self.uav.latest_camera_image
        if img is not None:
            surf = self._carla_image_to_surface(
                img, (UAV_PIP.width, UAV_PIP.height))
            self.screen.blit(surf, UAV_PIP.topleft)
        else:
            txt = self.font_sm.render("UAV cam initializing...", True, COLOR_DIM)
            self.screen.blit(txt, txt.get_rect(center=UAV_PIP.center))
        pygame.draw.rect(self.screen, COLOR_DEST, UAV_PIP, width=2)
        label = self.font_sm.render("UAV CAM", True, COLOR_DEST)
        self.screen.blit(label, (UAV_PIP.x + 6, UAV_PIP.y + 4))

    def _draw_sidebar(self) -> None:
        pygame.draw.rect(self.screen, COLOR_PANEL, SIDEBAR)
        pygame.draw.rect(self.screen, COLOR_BORDER, SIDEBAR, width=1)

        x = SIDEBAR.x + 14
        y = SIDEBAR.y + 12

        hdr = self.font_lg.render("SYSTEM STATE", True, COLOR_TEXT)
        self.screen.blit(hdr, (x, y))
        y += hdr.get_height() + 6

        state = self.coordination.current_state
        pygame.draw.circle(self.screen, _state_color(state), (x + 8, y + 12), 6)
        self.screen.blit(self.font_md.render(state, True, COLOR_TEXT), (x + 22, y + 4))
        y += 30

        ugv_loc = self.ugv.get_location()
        ugv_vel = self.ugv.get_velocity()
        uav_pos = self.uav.get_position()

        if ugv_loc:
            self.screen.blit(self.font_md.render(
                f"UGV  x={ugv_loc.x:7.1f}", True, COLOR_UGV), (x, y)); y += 18
            self.screen.blit(self.font_md.render(
                f"     y={ugv_loc.y:7.1f}", True, COLOR_UGV), (x, y)); y += 18
            self.screen.blit(self.font_md.render(
                f"     v={ugv_vel:5.1f} m/s", True, COLOR_UGV), (x, y)); y += 24

        self.screen.blit(self.font_md.render(
            f"UAV  x={uav_pos[0]:7.1f}", True, COLOR_UAV), (x, y)); y += 18
        self.screen.blit(self.font_md.render(
            f"     y={uav_pos[1]:7.1f}", True, COLOR_UAV), (x, y)); y += 18
        self.screen.blit(self.font_md.render(
            f"     z={uav_pos[2]:7.1f}", True, COLOR_UAV), (x, y)); y += 24

        if ugv_loc:
            dx = uav_pos[0] - ugv_loc.x
            dy = uav_pos[1] - ugv_loc.y
            dz = uav_pos[2] - ugv_loc.z
            sep = math.sqrt(dx * dx + dy * dy + dz * dz)
            self.screen.blit(self.font_md.render(
                f"Sep: {sep:5.1f} m", True, COLOR_TEXT), (x, y)); y += 30

        stats = self.coordination.get_statistics()
        self.screen.blit(self.font_sm.render(
            f"Waypoints: {stats.get('waypoints_generated', 0)}",
            True, COLOR_DIM), (x, y)); y += 16
        uptime = stats.get('uptime', 0) or 0
        mins, secs = divmod(int(uptime), 60)
        self.screen.blit(self.font_sm.render(
            f"Uptime:    {mins:02d}:{secs:02d}", True, COLOR_DIM), (x, y))
        y += 28

        # Camera target radio
        self.screen.blit(self.font_md.render("Main view:", True, COLOR_TEXT), (x, y))
        y += 22
        self._camera_radio_rects = []
        for tgt in self.CAMERA_TARGETS:
            row = pygame.Rect(x, y, SIDEBAR.width - 28, 22)
            checked = (tgt == self.camera_target)
            pygame.draw.circle(self.screen, COLOR_TEXT, (row.x + 8, row.y + 11), 7, 1)
            if checked:
                pygame.draw.circle(self.screen, COLOR_OK, (row.x + 8, row.y + 11), 4)
            label = self.font_md.render(self.CAMERA_LABELS[tgt], True, COLOR_TEXT)
            self.screen.blit(label, (row.x + 22, row.y + 3))
            self._camera_radio_rects.append((row, tgt))
            y += 24

        if self._latest_status_msg:
            y += 8
            self.screen.blit(self.font_sm.render("Last event:", True, COLOR_DIM), (x, y))
            y += 16
            words = self._latest_status_msg.split()
            line = ""
            wrap_w = SIDEBAR.width - 28
            for w in words:
                test = (line + " " + w).strip()
                if self.font_sm.size(test)[0] > wrap_w:
                    self.screen.blit(self.font_sm.render(line, True, COLOR_DIM), (x, y))
                    y += 14
                    line = w
                else:
                    line = test
            if line:
                self.screen.blit(self.font_sm.render(line, True, COLOR_DIM), (x, y))

        if self.is_paused:
            pause_txt = self.font_lg.render("PAUSED", True, COLOR_WARN)
            self.screen.blit(pause_txt, (
                SIDEBAR.x + SIDEBAR.width - pause_txt.get_width() - 14,
                SIDEBAR.y + SIDEBAR.height - pause_txt.get_height() - 14,
            ))

    def _draw_minimap(self) -> None:
        pygame.draw.rect(self.screen, COLOR_PANEL, MINIMAP)
        pygame.draw.rect(self.screen, COLOR_BORDER, MINIMAP, width=1)

        title = self.font_md.render(
            "MAP — click to set UGV destination", True, COLOR_TEXT)
        self.screen.blit(title, (MINIMAP.x + 12, MINIMAP.y + 6))

        inner = self._minimap_inner()
        pygame.draw.rect(self.screen, (24, 24, 30), inner)

        # Road network overlay (pre-rendered at init, just a blit per frame).
        self.screen.blit(self._road_overlay, MINIMAP.topleft)

        # Planned route (if scripted mode)
        try:
            wps = self.ugv.path_waypoints or []
        except Exception:
            wps = []
        if wps:
            pts = []
            for wp_tuple in wps:
                wp = wp_tuple[0] if isinstance(wp_tuple, tuple) else wp_tuple
                loc = wp.transform.location
                pts.append(self._world_to_minimap(loc.x, loc.y))
            if len(pts) >= 2:
                pygame.draw.lines(self.screen, COLOR_PATH, False, pts, 2)

        # UGV trail
        if len(self._ugv_trail) >= 2:
            trail_pts = [self._world_to_minimap(x, y) for x, y in self._ugv_trail]
            pygame.draw.lines(self.screen, COLOR_UGV, False, trail_pts, 1)

        # Detected obstacles
        for obs in self._latest_obstacles:
            world_pos = self._obstacle_to_world(obs)
            if world_pos is None:
                continue
            wx, wy, _wz = world_pos
            px, py = self._world_to_minimap(wx, wy)
            if not inner.collidepoint(px, py):
                continue
            pygame.draw.circle(self.screen, COLOR_OBSTACLE, (px, py), 3)

        # Destination marker (8-pt star)
        if self.coordination.final_destination:
            dx, dy, _dz = self.coordination.final_destination
            px, py = self._world_to_minimap(dx, dy)
            pygame.draw.polygon(self.screen, COLOR_DEST, [
                (px, py - 8), (px + 3, py - 3), (px + 8, py),
                (px + 3, py + 3), (px, py + 8), (px - 3, py + 3),
                (px - 8, py), (px - 3, py - 3),
            ])

        # UGV (triangle pointing along yaw)
        ugv_loc = self.ugv.get_location()
        if ugv_loc:
            tf = self.ugv.get_transform()
            yaw = tf.rotation.yaw if tf else 0.0
            self._draw_triangle(ugv_loc.x, ugv_loc.y, yaw, COLOR_UGV)

        # UAV (diamond)
        uav_pos = self.uav.get_position()
        self._draw_diamond(uav_pos[0], uav_pos[1], COLOR_UAV)

        # Cursor coords
        if self._cursor_world:
            cx, cy = self._cursor_world
            cur_txt = self.font_sm.render(
                f"cursor: x={cx:.1f}  y={cy:.1f}", True, COLOR_DIM)
            self.screen.blit(cur_txt, (MINIMAP.x + 12, MINIMAP.bottom - 22))

        # Legend
        legend_x = MINIMAP.right - 220
        legend_y = MINIMAP.y + 6
        for label, color, sx in (
            ("UGV", COLOR_UGV, 0), ("UAV", COLOR_UAV, 60),
            ("Dest", COLOR_DEST, 120), ("Path", COLOR_PATH, 180),
        ):
            pygame.draw.circle(self.screen, color, (legend_x + sx, legend_y + 8), 4)
            self.screen.blit(
                self.font_sm.render(label, True, COLOR_DIM),
                (legend_x + sx + 8, legend_y + 1))

    def _draw_triangle(self, wx: float, wy: float, yaw_deg: float, color) -> None:
        cx, cy = self._world_to_minimap(wx, wy)
        a = math.radians(yaw_deg)
        size = 7
        p1 = (cx + size * math.cos(a),       cy + size * math.sin(a))
        p2 = (cx + size * math.cos(a + 2.5), cy + size * math.sin(a + 2.5))
        p3 = (cx + size * math.cos(a - 2.5), cy + size * math.sin(a - 2.5))
        pygame.draw.polygon(self.screen, color, [p1, p2, p3])

    def _draw_diamond(self, wx: float, wy: float, color) -> None:
        cx, cy = self._world_to_minimap(wx, wy)
        size = 6
        pygame.draw.polygon(self.screen, color, [
            (cx, cy - size), (cx + size, cy), (cx, cy + size), (cx - size, cy)])

    def _draw_bottom_bar(self) -> None:
        pygame.draw.rect(self.screen, COLOR_PANEL, BOTTOM_BAR)
        pygame.draw.rect(self.screen, COLOR_BORDER, BOTTOM_BAR, width=1)
        hint = self.font_sm.render(
            "Left-click on the map to send the UGV to a new destination.",
            True, COLOR_DIM)
        self.screen.blit(hint, (12, BOTTOM_BAR.y + 18))
        for btn in self._buttons:
            btn.draw(self.screen, self.font_md)

    # ------------------------------------------------------------------ public
    def update(self) -> bool:
        """
        Pump events and render one frame. Call once per CARLA tick.

        Returns False if the operator has asked to quit (window close or Quit
        button), True otherwise.
        """
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            for btn in self._buttons:
                btn.handle(ev)
            if ev.type == pygame.MOUSEMOTION:
                self._cursor_world = (
                    self._minimap_to_world(*ev.pos)
                    if MINIMAP.collidepoint(ev.pos) else None
                )
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                # Camera-target radio takes priority over minimap
                clicked_radio = False
                for rect, tgt in self._camera_radio_rects:
                    if rect.collidepoint(ev.pos):
                        self.camera_target = tgt
                        clicked_radio = True
                        break
                if not clicked_radio and MINIMAP.collidepoint(ev.pos):
                    self._set_destination_from_click(*ev.pos)

        if self._quit_requested:
            return False

        try:
            self._update_chase_camera_transform()
        except Exception as e:
            print(f"[GUIConsole] chase cam transform error: {e}")

        # Sparse trail recording (don't burn memory at full tick rate)
        now = time.time()
        if now - self._last_trail_record_time > 0.25:
            loc = self.ugv.get_location()
            if loc:
                self._ugv_trail.append((loc.x, loc.y))
                if len(self._ugv_trail) > 800:
                    self._ugv_trail = self._ugv_trail[-800:]
                self._last_trail_record_time = now

        self.screen.fill(COLOR_BG)
        self._draw_main_view()
        self._draw_uav_pip()
        self._draw_sidebar()
        self._draw_minimap()
        self._draw_bottom_bar()
        pygame.display.flip()
        return True

    def cleanup(self) -> None:
        try:
            if self.chase_camera:
                self.chase_camera.stop()
                self.chase_camera.destroy()
        except Exception as e:
            print(f"[GUIConsole] chase cam destroy error: {e}")
        pygame.quit()
