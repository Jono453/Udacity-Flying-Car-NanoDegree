import argparse
import time
import msgpack
from enum import Enum, auto
import matplotlib.pyplot as plt

import numpy as np
import pandas as pd

from planning_utils import a_star, heuristic, create_grid
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
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
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

    # choose latitude and longitude in map (global) and transform
    # it to goal location in grid (local)          
    def choose_goal_global(self, goal_longitude,goal_latitude):   
        converted_global_goal = global_to_local([goal_longitude,goal_latitude,0],self.global_home)
        print("NED coordinates for specified goal: {0},{1},{2}".format(converted_global_goal[0],converted_global_goal[1],converted_global_goal[2]))        
        return converted_global_goal

    def point(self,p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(self,p1, p2, p3, epsilon=1e-6):   
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def prune_path(self,path):
        if path is not None:
            pruned_path = [p for p in path]
            i = 0
            while (i < len(pruned_path)-2):
                p1 = self.point(pruned_path[i])
                p2 = self.point(pruned_path[i+1])
                p3 = self.point(pruned_path[i+2])
                if self.collinearity_check(p1,p2,p3):
                    pruned_path.remove(pruned_path[i+1])
                else:
                    i += 1
        else:
            pruned_path = path
            
        return pruned_path

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        
        GOAL_LONGITUDE = -122.399466
        GOAL_LATITUDE = 37.795933
        TEST_GOAL_NORTH = 700.9
        TEST_GOAL_EAST = 266.0
        GRID_BOUNDARY = 921
        self.target_position[2] = TARGET_ALTITUDE

        global_data = pd.read_csv('colliders.csv')   
        parsed_latitude = str(global_data.columns[0]).split(' ')    
        #print(parsed_latitude[1])
        curr_gps_latitude = float(parsed_latitude[1])
        parsed_longitude = str(global_data.columns[1]).split(' ')    
        #print(parsed_longitude[2])
        curr_gps_longitude = float(parsed_longitude[2])

        # set global home position using colliders csv data
        self.set_home_position(curr_gps_longitude,curr_gps_latitude,-TARGET_ALTITUDE) 
        # (east_home, north_home, _, _) = utm.from_latlon(global_home[1], global_home[0])
        local_start = global_to_local([self._longitude,self._latitude,0], self.global_home)    
        #print("Local Start N,E: {0}.{1}".format(local_start[1],local_start[0]))                   
        
        print('global home {0}, global position {1}, local position {2}'.format(self.global_home,self.global_position,self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        print("Loaded obstacle data")
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        
        #print("North offset = {0}, east offset = {1}".format(north_offset, east_offset)) #North offset = -316, east offset = -445
        # Define starting point on the grid (originally this was just grid center) to be current local position from GPS
        grid_centre = np.array([-north_offset,-east_offset]) #coordinates in grid of 316,445
        grid_start = (int(local_start[0]-north_offset), int(local_start[1]-east_offset))                 

        debug_mode = False

        if (debug_mode):                        
            #grid_goal = (int(local_start[0]-north_offset+80), int(local_start[1]-east_offset+120))
            grid_goal = (int(TEST_GOAL_NORTH),int(TEST_GOAL_EAST))            
        else:                                       
            converted_global_goal = self.choose_goal_global(GOAL_LONGITUDE,GOAL_LATITUDE)
            if (converted_global_goal[0]-north_offset > GRID_BOUNDARY or converted_global_goal[1]-east_offset > GRID_BOUNDARY):
                print("Chosen goal (local) NED coordinate invalid")
                self.manual_transition()
            else:                
                print("Chosen goal (local) NED coordinate is valid")    
                grid_goal = (int(converted_global_goal[0]-north_offset),int(converted_global_goal[1]-east_offset)) #convert global grid coordinates
                # back to valid grid references
        
        print("Local Start: {0}".format(grid_start))
        #print("Global Start: {0},{1}".format(curr_gps_longitude,curr_gps_latitude))
        print("Local Goal: {0}".format(grid_goal))        
        #print("local_to_global Global Goal: {0}".format(local_to_global([grid_goal[1],grid_goal[0],0],self.global_home)))
        #print("set Global Goal: {0},{1}".format(GOAL_LONGITUDE,GOAL_LATITUDE)) #to check functionality        


        # Visualisation of local start and goal positions before A*
        plt.rcParams['figure.figsize'] = 9,9 # at runtime 
        plt.imshow(grid, origin='lower') 
        plt.xlabel('EAST')
        plt.ylabel('NORTH')  
        plt.plot(grid_centre[1], grid_centre[0], 'bo')        
        plt.plot(grid_start[1], grid_start[0], 'x') #E,N ordering
        plt.plot(grid_goal[1], grid_goal[0], 'o') #E,N ordering
        plt.show()

        # Run A* to find a path from start to goal        
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("Original A* path: {0}".format(len(path)))
        new_path = self.prune_path(path)
        print("Pruned path: {0}".format(len(new_path)))

        '''
        # Visualisation of pruned path
        if new_path is not None:
            pp = np.array(new_path)
            plt.plot(pp[:, 1], pp[:, 0], 'g')
            plt.scatter(pp[:, 1], pp[:, 0])
        '''

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in new_path]
        # Set self.waypoints
        self.waypoints = waypoints        
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
    np.set_printoptions(precision=3)
    time.sleep(1)    
    drone.start()
