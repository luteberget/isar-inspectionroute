

from typing import List, Tuple

from model import Location, RobotState, calculate_distance,calculate_line, mk_pose
from robotbase import RobotBase, TaskStatus
   
class VirtualRobot(RobotBase):
    current_state = None
    waypoints = None
    wp_step = 0
    next_wps = []
    step_n = 10
    events = []
    def __init__(self,initial_state) -> None:
        super().__init__()
        self.current_state = initial_state
    
    def set_plan(self,waypoints :List[Tuple[int, Location]]):
        self.waypoints = waypoints
        self.wp_step = 0
        self.next_wps = calculate_line(self.current_state.location,self.waypoints[self.wp_step](1),self.steps_n)
    
    def get_state(self):
        if len(self.waypoints) <= self.wp_step:
            print("Finished with given waypoints")
            return self.current_state
        elif self.current_state.location == self.waypoints[self.wp_step](1):
            # selecting new goal
            events += [TaskStatus(self.waypoints[self.wp_step](0),True)]
            self.wp_step = self.wp_step+1
            if len(self.waypoints) > self.wp_step:
                self.next_wps = calculate_line(self.current_state.location,self.waypoints[self.wp_step](1),self.step_n)
        elif len(self.next_wps) <= 0:
            # moving towards goal
            distance = calculate_distance(self.current_state.location,self.next_wps[0])
            self.current_state.location = self.next_wps[0]
            self.next_wps.pop(0)
            self.current_state.battery_constraint.remaining_distance -= distance
        else:
            events += [TaskStatus(self.waypoints[self.wp_step](0),False)]
            # Now : do not continoue on waypoints when one is failed.
            # Possible : continues and let's the 
        return self.current_state
    
    def get_event(self):
        evs = self.events
        self.events = []
        return evs