import argparse
import time
from enum import Enum
from datetime import datetime

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.flight_start_time = datetime.now()

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        # add callbacks for plotting (later using visdom)
        self.register_callback(MsgID.RAW_ACCELEROMETER, self.raw_accel_callback)
        self.register_callback(MsgID.BAROMETER, self.baro_callback)

    #Better visualisation of raw accelerometer and barometer(altitude) during box flight
    def raw_accel_callback(self):
        pass

    def baro_callback(self):
        pass

    """
    the current local position of the drone.
    Local position being defined as the NED position of the drone with respect to some (0,0,0) (the home position)
    """
    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """

        # in landing state, check safe height above ground
        if self.flight_state == States.LANDING:
           
            #if safe height above ground to begin disarm transition
            if (abs(self.local_position[2]) < 2.0):
                self.disarming_transition()
                print("Landing: {0}m".format(self.local_position[2]))
            else:
                print("Unsafe height to disarm")

        # in takeoff state, monitor height before starting first waypoint transition
        elif self.flight_state == States.TAKEOFF:

            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target to then start waypoints
            if altitude > 0.95 * self.target_position[2]:
                print("Starting waypoint sequence at safe altitude...")
                time.sleep(1)
                self.waypoint_transition()
       
    """
    The current velocity vector of the drone in meters/second, represented in the local NED frame
    """
    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """        
        # Check mission is loaded before any flight or arming can occur
        if self.flight_state == States.ARMING:
            self.calculate_box(args.box)
            print("Mission Loaded --- Success")
            time.sleep(0.5)
            self.takeoff_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission: #if not in mission state
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition() 
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()    
        elif self.flight_state == States.WAYPOINT:
            if self.guided:
                self.waypoint_transition()   
        elif self.flight_state == States.DISARMING:
            if (not self.armed) and (not self.guided):
                self.manual_transition()                        

    def calculate_box(self, boxSize):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        #N, E, altitude (set) for each waypoint
        # User defined array for mission waypoints including start waypoint
        mission = np.array([[boxSize, 0.0, args.height],[boxSize,boxSize,args.height],\
            [0.0, boxSize, args.height],[0.0, 0.0, args.height]])
        # Pass user mission into flight controller stored waypoints
        for mission_ctr in mission:
            self.all_waypoints.append(mission_ctr)
        return self.all_waypoints #filled in with desired mission as numpy 2D array (make dynamic)        

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming commandboxSize
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        # set the current location to be the home position (X,Y,Z)
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        time.sleep(1)
        self.flight_state = States.ARMING

    def takeoff_transition(self):        
        self.target_position[2] = args.height
        print("takeoff transition to {}m".format(self.target_position[2]))
        self.takeoff(self.target_position[2]) #drone takes off to desired altitude
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        
        # transition to waypoint state while there are still waypoints in list
        # else begin to land if at start position        
        if (len(self.all_waypoints) > 0):
            print("To Next WP")
            self.flight_state = States.WAYPOINT
            # extract from self.all_waypoints list and transfer to self.target_position
            curr_waypoint = self.all_waypoints.pop() #remove and return last item in waypoints list
            self.cmd_position(curr_waypoint[0],curr_waypoint[1],curr_waypoint[2],0) #NED,Heading
            print("Current WayPoint: ")
            print("North: {0}, East: {1}, Altitude: {2}".format(curr_waypoint[0],curr_waypoint[1],curr_waypoint[2]))       
            time.sleep(2)
        else:
            print("All waypoints completed")
            self.landing_transition()
        
    def landing_transition(self):
        print("start landing transition...")
        print("Current landing coordinates: {0},{1}".format(self.local_position[0],self.local_position[1]))        
        self.land()
        time.sleep(0.5)
        self.flight_state = States.LANDING

    def disarming_transition(self):      
        print("disarm transition")
        self.disarm
        print("Drone disarmed! Total Flight Time: {0}".format(datetime.now() - self.flight_start_time))
        self.flight_state = States.DISARMING       
        
    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        time.sleep(0.5)
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--height', type=int, default=10.0, help='target altitude for transition')
    parser.add_argument('--box', type=int, default=10.0, help='target box size')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
    print("Connected to drone at {0} port {1}".format(args.host, args.port))
