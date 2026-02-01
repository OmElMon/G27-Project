import carla
import random
import time
import sys
import os
# get the path to the agents folder so we can use basic agent and globalrouteplanner for scripted paths
# make sure your Carla folder is in the same directory as this file
curr_dir = os.path.dirname(os.path.abspath(__file__))
agent_dir = os.path.join(curr_dir, 'Carla/PythonAPI/carla')
sys.path.append(agent_dir)
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner

def main():
    '''
    Initial implementation of the UGV module of our system.
    The user can choose what type of navigation they want (full autopilot or scripted path)
    Simulation stops after UGV reaches the end of the scripted path or the user presses ctrl+c in the terminal
    author: Sean Bowden
    '''
    try:
        final_destination = None
        # connect to the environment
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0) # param in seconds
        # get world object
        world = client.get_world()
        print('Connected')
        # select a vehicle model
        # vehicle model catalogue: https://carla.readthedocs.io/en/latest/catalogue_vehicles/
        bp_lib = world.get_blueprint_library()
        ugv_bp = bp_lib.filter('vehicle.*')[0] # can change to specific vehicle with filter e.g. 'vehicle.jeep.wrangler_rubicon' instead of 'vehicle.*'
        # pick spawn point and spawn vehicle actor into world
        spawn_points = world.get_map().get_spawn_points()
        start_point = random.choice(spawn_points) # choosing a random spawn point from available spawn points
        ugv_vehicle = world.spawn_actor(ugv_bp, start_point)
        print(f'UGV spawned: {ugv_vehicle.type_id}')
        spectator = world.get_spectator()
        # let user decide navigation mode
        print('\nSelect Navigation Mode')
        print('1: Random Autopilot')
        print('2: Scripted Path (following a generated path)')
        nav_mode = input('Enter choice (1 or 2): ')
        
        # Scripted path logic
        if nav_mode == '2':
            # initialize agent (lights and signs off for demo purposes)
            agent = BasicAgent(ugv_vehicle)
            agent.ignore_traffic_lights(True)
            agent.ignore_stop_signs(True)
            agent.follow_speed_limits(False)
            #agent.set_target_speed(60.0)
            
            print('--- Generating scripted path ---')
            # get a random point for our destination
            points = world.get_map().get_spawn_points()
            # make sure we don't pick spot we are already on
            destinations = [p for p in points if p.location.distance(ugv_vehicle.get_location()) > 10.0]
            dest = random.choice(destinations)
            final_destination = dest.location
            # generate route to final destination
            grp = GlobalRoutePlanner(world.get_map(), 1.0)
            # trace_route returns list of tuples: (waypoint, RoadOption)
            path = grp.trace_route(ugv_vehicle.get_location(), final_destination)
            # draw path markers in server to ensure ugv is following correct path
            for wp, ro in path:
                world.debug.draw_string(wp.transform.location, 'X', color=carla.Color(r=255, g=0, b=0), life_time=90.0, persistent_lines=True)
                
            print(f'Path created with {len(path)} waypoints. UGV will follow red markers.')
            # force agent to use generated path
            agent.set_global_plan(path, stop_waypoint_creation=True)
            
        else:
            print('Random Autopilot enabled.')
            # autopilot required for both nav modes
            ugv_vehicle.set_autopilot(True, 8000)
            
        # attach spectator to UGV and adjust camera relative to vehicle's position
        transform = ugv_vehicle.get_transform()
        forward_vector = transform.get_forward_vector()
        offset = carla.Location(x=-20 * forward_vector.x, y=-20 * forward_vector.y, z=15)
        spectator_transform = carla.Transform(transform.location + offset, carla.Rotation(pitch=-30, yaw=transform.rotation.yaw, roll=transform.rotation.roll))
        spectator.set_transform(spectator_transform)
        # initialize timer for location data
        last_print_time = time.time()
        print('\nSimulation running. Press ctrl+c to stop simulation.')
        
        while True:
            world.wait_for_tick() # makes camera pov smooth
            # agent control logic
            if nav_mode == '2':
                if agent.done():
                    print('UGV has reached its final destination. Stopping simulation.')
                    break
                # get next control command from agent and apply to vehicle (throttle, brake, steer)
                ugv_vehicle.apply_control(agent.run_step())
                
            # spectator camera updates
            transform = ugv_vehicle.get_transform()
            forward_vector = transform.get_forward_vector()
            offset = carla.Location(x=-20 * forward_vector.x, y=-20 * forward_vector.y, z=15)
            spectator_transform = carla.Transform(transform.location + offset, carla.Rotation(pitch=-30, yaw=transform.rotation.yaw, roll=transform.rotation.roll))
            spectator.set_transform(spectator_transform)
            
            # print location data to console every t seconds
            curr_time = time.time()
            if curr_time - last_print_time > 3:
                location = ugv_vehicle.get_location()
                print(f'UGV location: x={location.x}, y={location.y}')
                last_print_time = curr_time
            
    except KeyboardInterrupt:
        print('Stopping simulation...')
    finally:
        # cleanup ugv actor after script ends
        # if not cleaned up, will live forever
        if 'ugv_vehicle' in locals():
            ugv_vehicle.destroy()
            print('UGV destroyed')
    
    
    
if __name__ == "__main__":
    main()