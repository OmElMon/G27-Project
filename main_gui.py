"""
UAV-UGV Coordination System — GUI Runner
=========================================
Parallel entry point that uses gui_console.GUIConsole instead of the
small UAV-only CameraDisplay window in main.py.

This file is a near-copy of main.py with the CameraDisplay-related code
swapped out for GUIConsole.

HOW TO RUN:
-----------
1. Start the CARLA server (CarlaUE4.exe).
2. python main_gui.py
3. Configure the simulation when prompted.
4. Click on the map in the bottom panel to manually re-route the UGV.
5. Use the radio buttons to switch the main camera between
   UGV chase / UAV chase / UAV onboard at runtime.
6. Press Quit (or close the window, or Ctrl+C) to stop.

KNOWN LIMITATIONS:
------------------
- Click-to-reroute only physically reroutes the UGV in scripted-path mode.
  In autopilot mode, the coordination platform's destination tracking is
  updated but the UGV will not change course.

Owner: Sean Bowden
"""

import sys
import time
import argparse
from typing import Optional

import carla

from config import (
    CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT,
    UAV_CONFIG, TOPICS, SystemState,
)
from message_broker import MessageBroker
from ugv_controller import UGVSubsystem, SpectatorController, NavigationMode
from uav_controller import UAVSubsystem
from coordination_platform import CoordinationPlatform
from data_logger import DataLogger
from gui_console import GUIConsole


class GUISimulationOrchestrator:
    """
    Orchestrator variant that drives a GUIConsole instead of the standalone
    CameraDisplay window. Mirrors SimulationOrchestrator from main.py with
    the GUI-specific bits swapped in.
    """

    def __init__(self):
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None

        self.broker = MessageBroker()

        self.ugv: Optional[UGVSubsystem] = None
        self.uav: Optional[UAVSubsystem] = None
        self.coordination: Optional[CoordinationPlatform] = None
        self.spectator: Optional[SpectatorController] = None
        self.gui: Optional[GUIConsole] = None
        self.data_logger: Optional[DataLogger] = None

        self.follow_distance = UAV_CONFIG['default_follow_distance']
        self.follow_altitude = UAV_CONFIG['default_altitude']
        self.navigation_mode = NavigationMode.SCRIPTED
        self.spectator_target = 'ugv'

        self.is_running = False
        self.simulation_start_time: Optional[float] = None

        self.stats = {
            'ticks': 0,
            'ugv_positions_received': 0,
            'uav_waypoints_generated': 0,
        }

        self.broker.subscribe(TOPICS['SYSTEM_STATUS'], self._on_system_status)

    def connect_to_carla(self) -> bool:
        print(f"Connecting to CARLA at {CARLA_HOST}:{CARLA_PORT}...")
        try:
            self.client = carla.Client(CARLA_HOST, CARLA_PORT)
            self.client.set_timeout(CARLA_TIMEOUT)
            self.world = self.client.get_world()
            print(f"Connected to map: {self.world.get_map().name}")
            return True
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            print("Make sure CARLA server is running")
            return False

    def configure_simulation(self) -> bool:
        print("\n" + "=" * 60)
        print("UAV-UGV COORDINATION SYSTEM (GUI MODE)")
        print("Florida Atlantic University - Senior Design Project")
        print("=" * 60)

        print("\nSelect UGV Navigation Mode:")
        print("  1: Scripted Path (recommended for click-to-reroute)")
        print("  2: Random Autopilot (click-to-reroute won't physically reroute)")

        choice = input("Enter choice (1 or 2) [default: 1]: ").strip()
        self.navigation_mode = (
            NavigationMode.AUTOPILOT if choice == '2' else NavigationMode.SCRIPTED
        )

        print(f"\nUAV Follow Parameters (press Enter for defaults):")
        try:
            dist_input = input(
                f"  Follow distance in meters [{self.follow_distance}]: ").strip()
            if dist_input:
                self.follow_distance = float(dist_input)
            alt_input = input(
                f"  Follow altitude in meters [{self.follow_altitude}]: ").strip()
            if alt_input:
                self.follow_altitude = float(alt_input)
        except ValueError:
            print("Invalid input, using defaults.")

        print("\nSelect initial main camera target (changeable at runtime):")
        print("  1: UGV chase  (ground vehicle perspective)")
        print("  2: UAV chase  (aerial perspective)")
        print("  3: UAV onboard (first-person drone view)")
        cam_choice = input("Enter choice (1, 2, or 3) [default: 1]: ").strip()
        self.spectator_target = (
            'uav_cam' if cam_choice == '3'
            else 'uav' if cam_choice == '2'
            else 'ugv'
        )

        print(f"\nConfiguration:")
        print(f"  Navigation Mode: {self.navigation_mode}")
        print(f"  Follow Distance: {self.follow_distance}m")
        print(f"  Follow Altitude: {self.follow_altitude}m")
        print(f"  Initial Camera:  {self.spectator_target}")

        return input("\nStart simulation? (y/n) [y]: ").strip().lower() != 'n'

    def initialize_subsystems(self) -> bool:
        print("\nInitializing subsystems...")

        print("\n[1/5] UGV Subsystem...")
        self.ugv = UGVSubsystem(self.world, self.broker)
        if not self.ugv.initialize():
            print("Failed to initialize UGV!")
            return False

        if self.navigation_mode == NavigationMode.SCRIPTED:
            self.ugv.set_navigation_mode(NavigationMode.SCRIPTED)
            self.ugv.draw_path_markers()
        else:
            self.ugv.set_navigation_mode(NavigationMode.AUTOPILOT)

        print("\n[2/5] Coordination Platform...")
        self.coordination = CoordinationPlatform(self.broker)
        self.coordination.set_parameters(
            follow_distance=self.follow_distance,
            altitude=self.follow_altitude,
        )
        if self.ugv.final_destination:
            dest = self.ugv.final_destination
            self.coordination.set_final_destination(dest.x, dest.y, dest.z)
        self.coordination.register_state_callback(self._on_state_change)

        print("\n[3/5] UAV Subsystem...")
        self.uav = UAVSubsystem(self.world, self.broker)
        ugv_loc = self.ugv.get_location()
        if ugv_loc is None:
            print("Could not get UGV location for UAV positioning!")
            return False
        if not self.uav.initialize(
            ugv_loc.x - self.follow_distance,
            ugv_loc.y,
            ugv_loc.z + self.follow_altitude,
        ):
            print("Failed to initialize UAV!")
            return False

        print("\n[4/5] Spectator + GUI Console...")
        # SpectatorController still drives the CarlaUE4 viewport as a backup.
        # The GUI window has its own dedicated chase camera.
        self.spectator = SpectatorController(self.world)
        self.gui = GUIConsole(
            self.world, self.broker,
            self.ugv, self.uav, self.coordination,
            initial_camera_target=self.spectator_target,
        )

        print("\n[5/5] Data Logger...")
        self.data_logger = DataLogger(self.broker)
        self.data_logger.start()

        print("\nAll subsystems initialized.")
        return True

    def run(self) -> None:
        print("\n" + "=" * 60)
        print("SIMULATION RUNNING (GUI mode)")
        print("Close the GUI window or press Ctrl+C to stop.")
        print("=" * 60 + "\n")

        self.is_running = True
        self.simulation_start_time = time.time()
        self.coordination.start()

        last_status_time = time.time()
        status_interval = 3.0

        try:
            while self.is_running:
                self.world.wait_for_tick()
                self.stats['ticks'] += 1

                # Soft pause: skip subsystem updates but keep the GUI alive so
                # the user can unpause. CARLA itself keeps ticking; this means
                # autopilot UGV will continue driving on its own.
                paused = bool(self.gui and self.gui.is_paused)

                if not paused:
                    ugv_ok = self.ugv.update()
                    state = self.coordination.update()
                    self.uav.update()
                    if self.data_logger:
                        self.data_logger.update()

                    # Drive the CarlaUE4 spectator viewport (backup view).
                    target = self.gui.camera_target if self.gui else self.spectator_target
                    if target == 'uav_cam':
                        import math
                        uav_state = self.uav.get_state()
                        loc = self.ugv.get_location()
                        if loc:
                            dx = loc.x - uav_state.x
                            dy = loc.y - uav_state.y
                            dz = loc.z - uav_state.z
                            hdist = math.sqrt(dx * dx + dy * dy)
                            yaw = math.degrees(math.atan2(dy, dx))
                            pitch = math.degrees(math.atan2(dz, max(hdist, 1e-3)))
                        else:
                            yaw = uav_state.yaw
                            pitch = -45
                        self.world.get_spectator().set_transform(carla.Transform(
                            carla.Location(x=uav_state.x, y=uav_state.y, z=uav_state.z - 1),
                            carla.Rotation(pitch=pitch, yaw=yaw, roll=0),
                        ))
                    elif target == 'uav':
                        self.spectator.follow_actor(self.uav.actor)
                    else:
                        self.spectator.follow_actor(self.ugv.vehicle)
                else:
                    ugv_ok = True
                    state = self.coordination.current_state

                # GUI runs even while paused so the user can unpause/quit.
                if self.gui and not self.gui.update():
                    print("\n[SIMULATION] GUI requested quit")
                    self.is_running = False
                    break

                if not paused:
                    if not ugv_ok and self.ugv.has_reached_destination():
                        print("\n[SIMULATION] UGV reached final destination!")
                        self.is_running = False
                        break

                    if state in (SystemState.SIMULATION_COMPLETE, SystemState.ERROR):
                        print(f"\n[SIMULATION] Ended with state: {state}")
                        self.is_running = False
                        break

                current_time = time.time()
                if current_time - last_status_time >= status_interval:
                    self._print_status()
                    last_status_time = current_time

        except KeyboardInterrupt:
            print("\n\n[SIMULATION] Interrupted by user")

        self.is_running = False

    def _print_status(self) -> None:
        ugv_loc = self.ugv.get_location()
        uav_pos = self.uav.get_position()
        if ugv_loc and uav_pos:
            import math
            dx = uav_pos[0] - ugv_loc.x
            dy = uav_pos[1] - ugv_loc.y
            dz = uav_pos[2] - ugv_loc.z
            separation = math.sqrt(dx * dx + dy * dy + dz * dz)
            horizontal_sep = math.sqrt(dx * dx + dy * dy)
        else:
            separation = 0
            horizontal_sep = 0

        elapsed = (
            time.time() - self.simulation_start_time
            if self.simulation_start_time else 0
        )
        print(f"[{elapsed:.1f}s] "
              f"State: {self.coordination.current_state} | "
              f"UGV vel: {self.ugv.get_velocity():.1f} m/s | "
              f"UAV->UGV: {horizontal_sep:.1f}m horiz, {separation:.1f}m total")

    def _on_state_change(self, old_state: str, new_state: str) -> None:
        print(f"\n*** STATE CHANGE: {old_state} -> {new_state} ***\n")

    def _on_system_status(self, message) -> None:
        pass

    def print_final_statistics(self) -> None:
        print("\n" + "=" * 60)
        print("SIMULATION STATISTICS")
        print("=" * 60)
        if self.simulation_start_time:
            print(f"Total Duration: {time.time() - self.simulation_start_time:.1f}s")
        print(f"Simulation Ticks: {self.stats['ticks']}")
        if self.coordination:
            cs = self.coordination.get_statistics()
            print(f"Waypoints Generated: {cs['waypoints_generated']}")
            print(f"Time Tracking: {cs['time_tracking']:.1f}s")
            print(f"Time Lost: {cs['time_lost']:.1f}s")
            print(f"Times Target Lost: {cs['times_lost']}")
            print(f"Times Reacquired: {cs['times_reacquired']}")
        bs = self.broker.get_statistics()
        print(f"Total Messages: {bs['messages_published']}")
        print("=" * 60)

    def cleanup(self) -> None:
        print("\nCleaning up...")
        if self.data_logger:
            self.data_logger.stop()
        if self.gui:
            self.gui.cleanup()
        if self.uav:
            self.uav.cleanup()
        if self.ugv:
            self.ugv.cleanup()
        print("Cleanup complete.")


def main():
    parser = argparse.ArgumentParser(description='UAV-UGV Coordination System (GUI)')
    parser.add_argument('--host', default=CARLA_HOST, help='CARLA server host')
    parser.add_argument('--port', type=int, default=CARLA_PORT, help='CARLA server port')
    parser.add_argument('--distance', type=float, default=None,
                        help='UAV follow distance (meters)')
    parser.add_argument('--altitude', type=float, default=None,
                        help='UAV follow altitude (meters)')
    args = parser.parse_args()

    orchestrator = GUISimulationOrchestrator()
    if args.distance:
        orchestrator.follow_distance = args.distance
    if args.altitude:
        orchestrator.follow_altitude = args.altitude

    try:
        if not orchestrator.connect_to_carla():
            sys.exit(1)
        if not orchestrator.configure_simulation():
            print("Simulation cancelled.")
            sys.exit(0)
        if not orchestrator.initialize_subsystems():
            print("Failed to initialize subsystems!")
            sys.exit(1)
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
