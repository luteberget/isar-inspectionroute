
from typing import List, Tuple
from alitra import Pose,Position,Orientation
from model import Location, RobotState,BatteryConstraint
from model import calculate_distance,calculate_rotation_distance,calculate_along_line, mk_pose
from robotbase import RobotBase, TaskStatus
from datetime import datetime
   
class VirtualRobot(RobotBase):
    current_state = None
    waypoints = []
    wp_step = 0
    speed = 0 #"m/s"
    rotation_speed = 0 
    time = 0
    events = []
    name = ""
    def __init__(self, robot_config):
        super().__init__()
        self.name = robot_config["name"]
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
        self.rotation_speed = float(robot_config["rotation_speed"])

    def set_plan(self,waypoints :List[Tuple[int, Location]]):
        self.waypoints = waypoints
        self.wp_step = 0
        self.time = datetime.now()
    def get_state(self):
        
        if len(self.waypoints) <= self.wp_step:
            ''' No current goal waypoint exists '''
            print("--")
            return self.current_state
        
        # Finds the next location for robot
        curr_time = datetime.now()
        curr_loc = self.current_state.current_location
        next_loc = self.waypoints[self.wp_step][1]
        distance = min(calculate_distance(curr_loc,next_loc),(curr_time-self.time).total_seconds()*self.speed)
        o_distance = min(calculate_rotation_distance(curr_loc,next_loc),(curr_time-self.time).total_seconds()*self.rotation_speed)
        self.current_state.current_location = calculate_along_line(curr_loc,next_loc,distance,o_distance)
        # Used battery
        self.current_state.battery_constraint.remaining_distance -= distance
        self.time = curr_time
        if distance == calculate_distance(curr_loc,next_loc) and o_distance == calculate_rotation_distance(curr_loc,next_loc):
            print("Starting new goal")
            self.events += [TaskStatus(self.waypoints[self.wp_step][0],True)]
            self.wp_step = self.wp_step + 1
        return self.current_state
    
    def get_event(self):
        evs = self.events
        self.events = []
        return evs
