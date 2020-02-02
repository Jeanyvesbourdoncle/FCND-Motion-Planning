import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils_grid import a_star, heuristic, create_grid, prune_path, plot_route
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.2:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)


		
###### FONCTION TO IMPLEMENT BETWEEN THE ARMING STATE AND THE TAKEOFF STATE#################
    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE
        
        # Time initialization to know how long we need to provide the path
        clock_system_init = time.time()
        
        # Read the latitude and the longitude from colliders.csv to know the initial starting point of the path
        data = np.loadtxt('colliders.csv', delimiter=';', dtype='str')
        field = data[0].split(",")
        lat = float(field[0].split()[1])
        lon = float(field[1].split()[1])
      
        # Set home position to (lon, lat, 0), the last parameter is for the altitude
        self.set_home_position(lon, lat, 0.0)
        
        # Retrieve current global position 
        current_global_position = [self._longitude, self._latitude, self._altitude]
        #print(current_global_position)
        
        # Convert from Geodetic (global position) to NED (local position) : use of the function global_to_local() in planning_utils_grid 
        current_local_position = global_to_local (current_global_position, self.global_home)
		#print(current_local_position)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        
        # Read in obstacle map, skip the 2 first lines
        data = np.loadtxt ('colliders.csv', delimiter =',', dtype= 'float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid   
        # Convert start position to current position rather than map center ( map center : north_offset and east_offset)
        grid_start = (int (current_local_position[0]-north_offset), int(current_local_position[1]-east_offset))
        
        # Set goal as some arbitrary position on the grid
        # 1-Convert current local position (NED) to global coordinates (Geodetic)
        goal_coord = local_to_global([current_local_position[0], current_local_position[1], current_local_position[2]], self.global_home)
        # 2- Add randomly latitude and longitude values in the goal location coordinates
        #Test 1
        goal_coord = (goal_coord[0]-0.0025, goal_coord[1] + 0.0035 , goal_coord[2] + 60)
        
		################OTHERS TESTS WITH RANDOMLY GOAL COORDINATES##########################
		# Test 2 : the goal is not reachable (obstacles)
        #goal_coord = (goal_coord[0]-0.0028, goal_coord[1] + 0.0040 , goal_coord[2] + 60)
        # Test 3 : the goal is outside of the card
        #goal_coord = (goal_coord[0]-0.0030, goal_coord[1] + 0.0045 , goal_coord[2] + 60)
        
        # Convert back the Genodetic Value to the local coordinates (NED) to send to the data on the simulator
        goal_pos = global_to_local(goal_coord, self.global_home)
        grid_goal = (int(goal_pos[0]-north_offset), int(goal_pos[1]-east_offset))
        
		#Safety Test : Default Case --> Landing Transition
        #Safety test : verify if the start and the goal are the same
        if ((grid_start[0] == grid_goal[0]) & (grid_start[1] == grid_goal[1])):
            print ("The goal coordinates are the same as the start coordinates")
            self.landing_transition() # Safety State
            return
        # Safety Test : verify if the goal coordinates are outside from the map
        if (grid_goal[0]<0 | grid_goal[1]<0 | grid_goal[0]>900 | grid_goal[1]>900):
            print("Overflow - the goal is not located in the map")
            self.landing_transition() # Safety State
            return
        # Safety Test : verify if the goal coordinates are inside a building
        if grid[grid_goal[0], grid_goal[1]] == 1 :
            print("Obstacles - the goal is located inside the building!")
            self.landing_transition() # Safety State
            return
		
		# Dignostics informations
        print ('Local Start : ', grid_start)
        print ('Local Goal :', grid_goal)
        
        # Run A* to find a path from start to goal + the cost of the path
        path, path_cost = a_star(grid, heuristic, grid_start, grid_goal)
        
		# Visualization without optimization (standard path)
        plot_route(grid, grid_start, grid_goal, path)
        print('length of path without optimization: {}'.format(len(path)))
        
        # Prune the path to minimize number of waypoints : the unneeded point are delete
        pruned_path = prune_path(path)
        
		# Visualization with the optimization (pruned path)
        plot_route(grid, grid_start,grid_goal,pruned_path)
        print('path length with optimization: {}'.format(len(pruned_path)))
        print('the cost of the path :{}'.format(path_cost))
        
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        print ('the waypoints for the path are :{}'.format(waypoints))
        print('Time to find the plan : {:.2f}'.format(time.time() - clock_system_init))
        self.waypoints = waypoints
        # Send waypoints to the simulator
        self.send_waypoints()


    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
