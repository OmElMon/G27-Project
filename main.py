"""
UAV-UGV Coordination System - Main Entry Point
===============================================
This is the main script that orchestrates all subsystems together.


HOW TO RUN:
----------
1. Start CARLA server: ./CarlaUE4.sh (Linux) or CarlaUE4.exe (Windows)
2. Run this script: python main.py
3. Follow the prompts to configure the simulation
4. Press Ctrl+C to stop

"""

import carla
import os
import time
import sys
import argparse
from typing import Optional, Tuple


def _find_carla_window_rect(title_substring: str = "carlaue4") -> Optional[Tuple[int, int, int, int]]:
    """
    Locate the CARLA simulator's top-level window and return its screen
    rect as (left, top, right, bottom). Matches by title substring
    (case-insensitive), requires the window to be visible and at least
    200x200 pixels to filter out splash/console windows. Windows only,
    returns None on other platforms or if no match is found.
    """
    if sys.platform != 'win32':
        return None

    import ctypes
    from ctypes import wintypes

    user32 = ctypes.windll.user32
    EnumWindows = user32.EnumWindows
    GetWindowTextW = user32.GetWindowTextW
    GetWindowTextLengthW = user32.GetWindowTextLengthW
    IsWindowVisible = user32.IsWindowVisible
    GetWindowRect = user32.GetWindowRect

    EnumWindowsProc = ctypes.WINFUNCTYPE(
        wintypes.BOOL, wintypes.HWND, wintypes.LPARAM
    )

    # Track the LARGEST matching window so we pick the main viewport over
    # any auxiliary dialogs/splash windows that contain "carla" in their title.
    best: dict = {'rect': None, 'area': 0}
    needle = title_substring.lower()

    def _callback(hwnd, _lparam):
        if not IsWindowVisible(hwnd):
            return True
        length = GetWindowTextLengthW(hwnd)
        if length == 0:
            return True
        buf = ctypes.create_unicode_buffer(length + 1)
        GetWindowTextW(hwnd, buf, length + 1)
        title = buf.value
        if needle not in title.lower():
            return True
        rect = wintypes.RECT()
        if not GetWindowRect(hwnd, ctypes.byref(rect)):
            return True
        w = rect.right - rect.left
        h = rect.bottom - rect.top
        if w < 200 or h < 200:
            return True
        area = w * h
        if area > best['area']:
            best['area'] = area
            best['rect'] = (rect.left, rect.top, rect.right, rect.bottom)
            best['title'] = title
        return True

    EnumWindows(EnumWindowsProc(_callback), 0)
    if best['rect'] is not None:
        print(f"[CameraDisplay] Found CARLA window '{best.get('title')}' at {best['rect']}")
    return best['rect']


def _get_primary_screen_size() -> Tuple[int, int]:
    """
    Return (width, height) of the primary monitor in pixels using Win32
    GetSystemMetrics (no pygame dependency, so it's safe to call before
    pygame.init()). Falls back to a 1920x1080 guess on non-Windows systems.
    """
    if sys.platform != 'win32':
        return (1920, 1080)
    import ctypes
    user32 = ctypes.windll.user32
    # SM_CXSCREEN = 0, SM_CYSCREEN = 1 — primary monitor dimensions.
    return (user32.GetSystemMetrics(0), user32.GetSystemMetrics(1))


def _raise_pygame_window() -> None:
    """
    Bring the most recently created SDL/pygame window to the foreground
    on Windows so it isn't hidden behind the CARLA viewport. No-op on
    other platforms or if the window can't be located.
    """
    if sys.platform != 'win32':
        return
    try:
        import ctypes
        from ctypes import wintypes
        user32 = ctypes.windll.user32
        # Find the pygame window by its caption.
        hwnd = user32.FindWindowW(None, "UAV Camera Feed")
        if not hwnd:
            return
        HWND_TOPMOST = wintypes.HWND(-1)
        HWND_NOTOPMOST = wintypes.HWND(-2)
        SWP_NOMOVE = 0x0002
        SWP_NOSIZE = 0x0001
        SWP_SHOWWINDOW = 0x0040
        # Briefly mark topmost to force it above CARLA, then release so it
        # behaves like a normal window the user can click behind.
        user32.SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0,
                            SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW)
        user32.SetWindowPos(hwnd, HWND_NOTOPMOST, 0, 0, 0, 0,
                            SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW)
    except Exception as e:
        print(f"[CameraDisplay] Could not raise pygame window: {e}")

# Optional dependencies for UAV camera display window
try:
    import numpy as np
    import pygame
    DISPLAY_AVAILABLE = True
except ImportError:
    DISPLAY_AVAILABLE = False

# Import our subsystems
from config import (
    CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT,
    UAV_CONFIG, UGV_CONFIG, COORDINATION_CONFIG,
    TOPICS, SystemState
)
from message_broker import MessageBroker
from ugv_controller import UGVSubsystem, SpectatorController, NavigationMode
from uav_controller import UAVSubsystem
from coordination_platform import CoordinationPlatform
from data_logger import DataLogger


class CameraDisplay:
    """
    Displays the UAV's onboard camera feed in a separate pygame window.

    Since CARLA's spectator viewport can't be overlaid with custom graphics,
    this creates a standalone window showing the drone's camera sensor output.
    Acts as a picture-in-picture view when using other camera perspectives.
    """

    def __init__(
        self,
        width: int = 320,
        height: int = 240,
        position: Optional[Tuple[int, int]] = None,
        margin: int = 20,
    ):
        """
        Initialize the pygame display window.

        Args:
            width, height: Window size in pixels
            position: Explicit (x, y) top-left screen coordinates. If None,
                the window is anchored to the top-right of the primary
                display so it overlays the CARLA sim window.
            margin: Pixel gap from the screen edge when auto-anchoring.
        """
        # SDL's video subsystem reads SDL_VIDEO_WINDOW_POS during init, NOT
        # at window creation time. This means the env var must be set before
        # pygame.init() runs, otherwise SDL falls back to its default
        # "center on primary monitor" placement. We therefore compute the
        # target coordinates using only ctypes/Win32 (no pygame) first, then
        # set the env var, then finally initialize pygame.
        if position is not None:
            x, y = position
        else:
            carla_rect = _find_carla_window_rect("carlaue4")
            if carla_rect is not None:
                left, top, right, _bottom = carla_rect
                x = max(left, right - width - margin)
                y = top + margin
            else:
                screen_w, screen_h = _get_primary_screen_size()
                x = max(0, screen_w - width - margin)
                y = margin

        os.environ['SDL_VIDEO_WINDOW_POS'] = f"{x},{y}"
        print(f"[CameraDisplay] Placing pygame window at ({x}, {y}) size {width}x{height}")

        # If pygame's video subsystem was already initialized by earlier code,
        # cycle it so SDL re-reads the env var. init() is a no-op if already
        # initialized, so we do an explicit display.quit() first to be safe.
        pygame.display.quit()
        pygame.init()

        self.width = width
        self.height = height
        self.display = pygame.display.set_mode(
            (width, height),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        pygame.display.set_caption("UAV Camera Feed")

        # CARLA typically owns the foreground after it launches, so the newly
        # created pygame window can end up behind it. Force it above CARLA.
        _raise_pygame_window()

    def update(self, carla_image) -> bool:
        """
        Render a CARLA camera image to the pygame window.

        Args:
            carla_image: carla.Image from the UAV camera sensor

        Returns:
            True if window is still open, False if user closed it
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

        if carla_image is None:
            return True

        # Convert CARLA image (BGRA bytes) to a pygame surface
        array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
        array = array.reshape((carla_image.height, carla_image.width, 4))
        array = array[:, :, :3][:, :, ::-1]  # BGRA -> RGB

        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        # Scale to window size if camera resolution differs
        if surface.get_size() != (self.width, self.height):
            surface = pygame.transform.scale(surface, (self.width, self.height))

        self.display.blit(surface, (0, 0))
        pygame.display.flip()

        return True

    def cleanup(self):
        """Shut down pygame."""
        pygame.quit()


class SimulationOrchestrator:
    """
    Main orchestrator that manages the entire simulation.
    
    This class:
    1. Initializes all subsystems
    2. Manages the simulation lifecycle
    3. Handles user input and configuration
    4. Coordinates shutdown and cleanup
    """
    
    def __init__(self):
        """Initialize the orchestrator."""
        # CARLA connection
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        
        # Communication
        self.broker = MessageBroker()
        
        # Subsystems
        self.ugv: Optional[UGVSubsystem] = None
        self.uav: Optional[UAVSubsystem] = None
        self.coordination: Optional[CoordinationPlatform] = None
        self.spectator: Optional[SpectatorController] = None
        self.camera_display: Optional[CameraDisplay] = None
        self.data_logger: Optional[DataLogger] = None

        # Configuration
        self.follow_distance = UAV_CONFIG['default_follow_distance']
        self.follow_altitude = UAV_CONFIG['default_altitude']
        self.navigation_mode = NavigationMode.SCRIPTED
        self.spectator_target = 'ugv'  # 'ugv', 'uav', or 'uav_cam'

        # State
        self.is_running = False
        self.simulation_start_time: Optional[float] = None
        
        # Statistics tracking
        self.stats = {
            'ticks': 0,
            'ugv_positions_received': 0,
            'uav_waypoints_generated': 0,
        }
        
        # Subscribe to system messages for logging
        self.broker.subscribe(TOPICS['SYSTEM_STATUS'], self._on_system_status)
    
    def connect_to_carla(self) -> bool:
        """
        Establish connection to CARLA server.
        
        Returns:
            True if connected successfully, False otherwise
        """
        print(f"Connecting to CARLA at {CARLA_HOST}:{CARLA_PORT}...")
        
        try:
            self.client = carla.Client(CARLA_HOST, CARLA_PORT)
            self.client.set_timeout(CARLA_TIMEOUT)
            self.world = self.client.get_world()
            
            # Print world info
            map_name = self.world.get_map().name
            print(f"Connected to map: {map_name}")
            
            return True
            
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            print("Make sure CARLA server is running")
            return False
    
    def configure_simulation(self) -> bool:
        """
        Interactive configuration of simulation parameters.
        
        Returns:
            True if configuration successful, False if cancelled
        """
        print("\n" + "="*60)
        print("UAV-UGV COORDINATION SYSTEM")
        print("Florida Atlantic University - Senior Design Project")
        print("="*60)
        
        # Navigation mode selection
        print("\nSelect UGV Navigation Mode:")
        print("  1: Scripted Path (UGV follows a generated route)")
        print("  2: Random Autopilot (UGV drives randomly)")
        
        choice = input("Enter choice (1 or 2) [default: 1]: ").strip()
        
        if choice == '2':
            self.navigation_mode = NavigationMode.AUTOPILOT
        else:
            self.navigation_mode = NavigationMode.SCRIPTED
        
        # UAV follow parameters
        print(f"\nUAV Follow Parameters (press Enter for defaults):")
        
        try:
            dist_input = input(f"  Follow distance in meters [{self.follow_distance}]: ").strip()
            if dist_input:
                self.follow_distance = float(dist_input)
            
            alt_input = input(f"  Follow altitude in meters [{self.follow_altitude}]: ").strip()
            if alt_input:
                self.follow_altitude = float(alt_input)
        except ValueError:
            print("Invalid input, using defaults.")

        # Spectator camera selection
        print(f"\nSelect Spectator Camera Target:")
        print("  1: Follow UGV (ground vehicle perspective)")
        print("  2: Follow UAV (aerial perspective)")
        print("  3: UAV Onboard Camera (first-person drone view)")

        cam_choice = input("Enter choice (1, 2, or 3) [default: 1]: ").strip()
        if cam_choice == '3':
            self.spectator_target = 'uav_cam'
        elif cam_choice == '2':
            self.spectator_target = 'uav'
        else:
            self.spectator_target = 'ugv'

        camera_labels = {'ugv': 'UGV', 'uav': 'UAV', 'uav_cam': 'UAV Onboard Camera'}
        print(f"\nConfiguration:")
        print(f"  Navigation Mode: {self.navigation_mode}")
        print(f"  Follow Distance: {self.follow_distance}m")
        print(f"  Follow Altitude: {self.follow_altitude}m")
        print(f"  Spectator Camera: {camera_labels[self.spectator_target]}")
        
        confirm = input("\nStart simulation? (y/n) [y]: ").strip().lower()
        return confirm != 'n'
    
    def initialize_subsystems(self) -> bool:
        """
        Initialize all subsystems.
        
        Returns:
            True if all subsystems initialized successfully
        """
        print("\nInitializing subsystems...")
        
        # === UGV SUBSYSTEM ===
        print("\n[1/4] Initializing UGV Subsystem...")
        self.ugv = UGVSubsystem(self.world, self.broker)
        
        if not self.ugv.initialize():
            print("Failed to initialize UGV!")
            return False
        
        # Set navigation mode
        if self.navigation_mode == NavigationMode.SCRIPTED:
            self.ugv.set_navigation_mode(NavigationMode.SCRIPTED)
            self.ugv.draw_path_markers()
        else:
            self.ugv.set_navigation_mode(NavigationMode.AUTOPILOT)
        
        # === COORDINATION PLATFORM ===
        print("\n[2/4] Initializing Coordination Platform...")
        self.coordination = CoordinationPlatform(self.broker)
        self.coordination.set_parameters(
            follow_distance=self.follow_distance,
            altitude=self.follow_altitude
        )
        
        # Set final destination if using scripted path
        if self.ugv.final_destination:
            dest = self.ugv.final_destination
            self.coordination.set_final_destination(dest.x, dest.y, dest.z)
        
        # Register state change callback
        self.coordination.register_state_callback(self._on_state_change)
        
        # === UAV SUBSYSTEM ===
        print("\n[3/4] Initializing UAV Subsystem...")
        self.uav = UAVSubsystem(self.world, self.broker)
        
        # Start UAV above the UGV's starting position
        ugv_loc = self.ugv.get_location()
        if ugv_loc:
            uav_start_x = ugv_loc.x - self.follow_distance
            uav_start_y = ugv_loc.y
            uav_start_z = ugv_loc.z + self.follow_altitude
            
            if not self.uav.initialize(uav_start_x, uav_start_y, uav_start_z):
                print("Failed to initialize UAV!")
                return False
        else:
            print("Could not get UGV location for UAV positioning!")
            return False
        
        # === SPECTATOR CAMERA ===
        print("\n[4/5] Setting up spectator camera...")
        self.spectator = SpectatorController(self.world)

        # === UAV CAMERA DISPLAY (optional) ===
        if DISPLAY_AVAILABLE:
            self.camera_display = CameraDisplay()
            print("UAV camera display window opened")
        else:
            print("Note: Install pygame and numpy for UAV camera display window")
            print("  pip install pygame numpy")

        # === DATA LOGGER ===
        print("\n[5/5] Initializing Data Logger...")
        self.data_logger = DataLogger(self.broker)
        self.data_logger.start()

        print("\nAll subsystems initialized successfully!")
        return True
    
    def run(self) -> None:
        """
        Main simulation loop.
        
        This runs until:
        - User presses Ctrl+C
        - UGV reaches destination
        - Simulation enters COMPLETE or ERROR state
        """
        print("\n" + "="*60)
        print("SIMULATION RUNNING")
        print("Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        self.is_running = True
        self.simulation_start_time = time.time()
        
        # Start coordination platform
        self.coordination.start()
        
        # Timing for status prints
        last_status_time = time.time()
        status_interval = 3.0  # Print status every 3 seconds
        
        try:
            while self.is_running:
                self.world.wait_for_tick()
                self.stats['ticks'] += 1
                
                # Update UGV (may return False if destination reached)
                ugv_ok = self.ugv.update()
                
                state = self.coordination.update()
                
                self.uav.update()

                # === UPDATE DATA LOGGER ===
                if self.data_logger:
                    self.data_logger.update()

                # === UPDATE SPECTATOR CAMERA ===
                if self.spectator_target == 'uav_cam':
                    # First-person drone camera looking down at the UGV
                    import math
                    uav_state = self.uav.get_state()
                    ugv_loc = self.ugv.get_location()
                    if ugv_loc:
                        # Calculate look-at direction from UAV toward UGV
                        dx = ugv_loc.x - uav_state.x
                        dy = ugv_loc.y - uav_state.y
                        dz = ugv_loc.z - uav_state.z
                        hdist = math.sqrt(dx**2 + dy**2)
                        look_yaw = math.degrees(math.atan2(dy, dx))
                        look_pitch = math.degrees(math.atan2(dz, hdist))
                    else:
                        look_yaw = uav_state.yaw
                        look_pitch = -45
                    cam_transform = carla.Transform(
                        carla.Location(x=uav_state.x, y=uav_state.y, z=uav_state.z - 1),
                        carla.Rotation(pitch=look_pitch, yaw=look_yaw, roll=0)
                    )
                    self.world.get_spectator().set_transform(cam_transform)
                elif self.spectator_target == 'uav':
                    self.spectator.follow_actor(self.uav.actor)
                else:
                    self.spectator.follow_actor(self.ugv.vehicle)

                # Update UAV camera feed display
                if self.camera_display:
                    if not self.camera_display.update(self.uav.latest_camera_image):
                        # User closed the pygame window
                        self.camera_display = None
                
                # === CHECK TERMINATION CONDITIONS ===
                
                # UGV reached destination
                if not ugv_ok and self.ugv.has_reached_destination():
                    print("\n[SIMULATION] UGV reached final destination!")
                    self.is_running = False
                    break
                
                # Coordination platform says we're done
                if state in [SystemState.SIMULATION_COMPLETE, SystemState.ERROR]:
                    print(f"\n[SIMULATION] Ended with state: {state}")
                    self.is_running = False
                    break
                
                # === PERIODIC STATUS UPDATES ===
                current_time = time.time()
                if current_time - last_status_time >= status_interval:
                    self._print_status()
                    last_status_time = current_time
                    
        except KeyboardInterrupt:
            print("\n\n[SIMULATION] Interrupted by user")
        
        self.is_running = False
    
    def _print_status(self) -> None:
        """Print current simulation status."""
        ugv_loc = self.ugv.get_location()
        uav_pos = self.uav.get_position()
        
        if ugv_loc and uav_pos:
            import math
            dx = uav_pos[0] - ugv_loc.x
            dy = uav_pos[1] - ugv_loc.y
            dz = uav_pos[2] - ugv_loc.z
            separation = math.sqrt(dx**2 + dy**2 + dz**2)
            horizontal_sep = math.sqrt(dx**2 + dy**2)
        else:
            separation = 0
            horizontal_sep = 0
        
        elapsed = time.time() - self.simulation_start_time if self.simulation_start_time else 0
        
        print(f"[{elapsed:.1f}s] "
              f"State: {self.coordination.current_state} | "
              f"UGV vel: {self.ugv.get_velocity():.1f} m/s | "
              f"UAV->UGV: {horizontal_sep:.1f}m horiz, {separation:.1f}m total")
    
    def _on_state_change(self, old_state: str, new_state: str) -> None:
        """Callback when coordination state changes."""
        print(f"\n*** STATE CHANGE: {old_state} -> {new_state} ***\n")
        
        if new_state == SystemState.TARGET_LOST:
            print("  UAV has lost track of UGV!")
            print("  Entering search mode...")
        elif new_state == SystemState.TRACKING and old_state == SystemState.TARGET_LOST:
            print("  UGV position reacquired!")
            print("  Resuming tracking...")
    
    def _on_system_status(self, message) -> None:
        """Callback for system status messages."""
        pass
    
    def print_final_statistics(self) -> None:
        """Print final simulation statistics."""
        print("\n" + "="*60)
        print("SIMULATION STATISTICS")
        print("="*60)
        
        if self.simulation_start_time:
            elapsed = time.time() - self.simulation_start_time
            print(f"Total Duration: {elapsed:.1f} seconds")
        
        print(f"Simulation Ticks: {self.stats['ticks']}")
        
        if self.coordination:
            coord_stats = self.coordination.get_statistics()
            print(f"Waypoints Generated: {coord_stats['waypoints_generated']}")
            print(f"Time Tracking: {coord_stats['time_tracking']:.1f}s")
            print(f"Time Lost: {coord_stats['time_lost']:.1f}s")
            print(f"Times Target Lost: {coord_stats['times_lost']}")
            print(f"Times Reacquired: {coord_stats['times_reacquired']}")
        
        broker_stats = self.broker.get_statistics()
        print(f"Total Messages: {broker_stats['messages_published']}")
        
        print("="*60)
    
    def cleanup(self) -> None:
        """Clean up all resources."""
        print("\nCleaning up...")

        if self.data_logger:
             self.data_logger.stop()

        if self.camera_display:
            self.camera_display.cleanup()

        if self.uav:
            self.uav.cleanup()

        if self.ugv:
            self.ugv.cleanup()

        print("Cleanup complete!")


def main():
    """Main entry point."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='UAV-UGV Coordination System'
    )
    parser.add_argument(
        '--host', 
        default=CARLA_HOST,
        help='CARLA server host'
    )
    parser.add_argument(
        '--port', 
        type=int, 
        default=CARLA_PORT,
        help='CARLA server port'
    )
    parser.add_argument(
        '--distance',
        type=float,
        default=None,
        help='UAV follow distance (meters)'
    )
    parser.add_argument(
        '--altitude',
        type=float,
        default=None,
        help='UAV follow altitude (meters)'
    )
    
    args = parser.parse_args()
    
    # Create orchestrator
    orchestrator = SimulationOrchestrator()
    
    # Override defaults from command line
    if args.distance:
        orchestrator.follow_distance = args.distance
    if args.altitude:
        orchestrator.follow_altitude = args.altitude
    
    try:
        # Connect to CARLA
        if not orchestrator.connect_to_carla():
            sys.exit(1)
        
        # Configure simulation
        if not orchestrator.configure_simulation():
            print("Simulation cancelled.")
            sys.exit(0)
        
        # Initialize subsystems
        if not orchestrator.initialize_subsystems():
            print("Failed to initialize subsystems!")
            sys.exit(1)
        
        # Run simulation
        orchestrator.run()
        orchestrator.print_final_statistics()
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        orchestrator.cleanup()


if __name__ == '__main__':
    main()
