
from typing import List, Tuple
from alitra import Pose,Position,Orientation
from model import Location, RobotState, calculate_distance,calculate_line,BatteryConstraint, mk_pose
from robotbase import RobotBase, TaskStatus
from datetime import datetime
   
class VirtualRobot(RobotBase):
    current_state = None
    waypoints = []
    wp_step = 0
    #next_wps = []
    #step_n = 10
    speed = 0 #"m/s"
    time = 0
    events = []

    def __init__(self, robot_config):
        super().__init__()
        init = robot_config["initial_state"]
        pos = init["location"]["position"]
        position = Position(float(pos["x"]),float(pos["y"]),float(pos["z"]),"")
        ori = init["location"]["orientation"]
        orientation = Orientation(float(ori["x"]),float(ori["y"]),float(ori["z"]),float(ori["w"]),"")
        loc = Location(init["location"]["name"],Pose(position,orientation,""))
        batt = init["battery_constraint"]
        battery = BatteryConstraint(loc,float(batt["battery_distance"]),float(batt["remaining_distance"]))
        self.current_state = RobotState(loc,battery)
        self.speed = float(robot_config["speed"])

    def set_plan(self,waypoints :List[Tuple[int, Location]]):
        self.waypoints = waypoints
        self.wp_step = 0
        self.time = datetime.now()
        #self.next_wps = calculate_line(self.current_state.location,self.waypoints[self.wp_step](1),self.steps_n)
    
    def get_state(self):
        if len(self.waypoints) <= self.wp_step:
            print("Finished with given waypoints")
            return self.current_state
        elif self.current_state.location == self.waypoints[self.wp_step](1):
            # selecting new goal
            events += [TaskStatus(self.waypoints[self.wp_step](0),True)]
            self.wp_step = self.wp_step+1
            self.time = datetime.now()
            if len(self.waypoints) > self.wp_step:
                self.next_wps = calculate_line(self.current_state.location,self.waypoints[self.wp_step](1),self.step_n)
        else:
            # moving towards goal
            self.next_location = 0
            distance = calculate_distance(self.current_state.location,self.next_wps[0])
            self.current_state.location = self.next_wps[0]
            self.next_wps.pop(0)
            self.current_state.battery_constraint.remaining_distance -= distance
        else:
            #events += [TaskStatus(self.waypoints[self.wp_step](0),False)]
            # Now : do not continoue on waypoints when one is failed.
            # Possible : continues and let's the 
        return self.current_state
    
    def get_event(self):
        evs = self.events
        self.events = []
        return evs
