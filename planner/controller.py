import json
import time

from typing import List, Optional
from isarrobot import ISARRobot

from model import (
    RobotState,
    Status,
    Waypoint,
    WaypointStatus,
    integrate_robot_plan,
    json_dumps_dataclass,
)

from planner_greedy import greedy_sequence
from robotbase import RobotBase

class VirtualRobot:
    pass


class Controller:
    waypoints: List[Waypoint] = []
    robots :List[RobotBase] = []
    last_status_message: Optional[str] = None
    current_plan_progress :List[int] = []
    current_plan :List[List[int]] = []
    current_plan_version :int = 0

    def __init__(self, configuration):
        print("Loading configuration:", json.dumps(configuration, indent=2))

        self.planner_name = configuration["planner"]
        if configuration["planner"] == "greedy":
            self.planner_fn = greedy_sequence
        elif configuration["planner"] == "vrp":
            raise Exception("not implemented")
        else:
            raise Exception("unknown planner")
        
        for robot_configuration in configuration["robots"]:
            if robot_configuration["type"] == "isar":
                self.robots.append(ISARRobot(robot_configuration))
            elif robot_configuration["type"] == "virtual":
                self.robots.append(VirtualRobot(robot_configuration))
            else:
                raise Exception("Unknown robot type")
            
            self.current_plan.append([])
            self.current_plan_progress.append(0)

        self.publish_waypoints()
        self.publish_plan(0.0, [])

    def robot_ready(self, state :RobotState):
        return state.current_location is not None

    def main_loop(self):
        while True:
            robot_states = [robot.get_state() for robot in self.robots]
            self.publish_plan(robot_states)

            self.updates_from_robot_events()

            if not all(self.robot_ready(s) for s in robot_states):
                self.set_status("Waiting for robot state information.")
            else:
                plan_flaw = self.is_plan_valid(robot_states)
                if plan_flaw is not None:
                    self.log_msg("Updating plan because of flaw: " + plan_flaw)
                    self.update_plan(robot_states)
                    self.publish_plan(robot_states)

            time.sleep(0.5)

    def updates_from_robot_events(self):
        for robot in self.robots:
            for task_status in robot.get_event():
                if task_status.waypoint >= len(self.waypoints):
                    self.log_msg(f"WARNING: unknown waypoint id {task_status.waypoint}") 
                    continue
                    
                    # Mark the waypoint in the waypoint list.
                if task_status.success:
                    self.log_msg(f"Successfully inspected waypoint {task_status.waypoint}")
                    self.waypoints[task_status.waypoint].status = WaypointStatus.SUCCESS
                else:
                    self.log_msg(f"Failed to inspected waypoint {task_status.waypoint}")
                    self.waypoints[task_status.waypoint].status = WaypointStatus.FAILURE

                    # Update the progress of the robot.
                for robot_idx in range(len(self.robots)):
                    robot_plan = self.current_plan[robot_idx]
                    if task_status.waypoint in robot_plan:
                        plan_idx = robot_plan.index(task_status.waypoint)
                        progress = self.current_plan_progress[robot_idx]
                        if plan_idx != progress:
                            self.log_msg(f"Warning: robot finished task that was not the first planned.")
                            robot_plan[plan_idx], robot_plan[progress] = robot_plan[progress], robot_plan[plan_idx]
                            
                        self.current_plan_progress[robot_idx] += 1

    def log_msg(self, msg):
        self._mqtt_client.publish("planner/message", msg, retain=False)
        print(msg)

    def is_plan_valid(self, robot_states :List[RobotState]):
        pending_waypoints = frozenset((i for i,wp in enumerate(self.waypoints) if wp.status == WaypointStatus.PENDING))
        planned_waypoints = frozenset((i for robot_plan in self.current_plan for i in robot_plan))
        
        if pending_waypoints != planned_waypoints:
            return "New waypoint added."

        _,plan = self.integrate_plan(robot_states)
        if min((x.remaining_battery for r in plan for x in r), default=1.0) < 0.0:
            return("Not enough battery to finish plan.")

        # Plan is valid.
        return None

    def update_plan(self, robot_states :List[RobotState]):
        new_plan = self.planner_fn(robot_states, self.waypoints)
        self.current_plan = new_plan
        self.current_plan_progress = [0 for _ in new_plan]
        self.current_plan_version += 1

    def set_status(self, s):
        if self.last_status_message == s:
            return
        self.last_status_message = s
        print("Status: ", s)

    def publish_plan(self, robot_states :List[RobotState]):
        total_cost, combined_plan = self.integrate_plan(robot_states)
        status = Status(self.planner_name, total_cost, combined_plan)
        json_output = json_dumps_dataclass(status)
        self._mqtt_client.publish("planner/plan", json_output, retain=True)

    def integrate_plan(self, robot_states):
        total_cost = 0.0
        combined_plan = []
        for robot_state, robot_plan, start_idx in zip(robot_states, self.current_plan, self.current_plan_progress):
            cost, plan = integrate_robot_plan(robot_state, [self.waypoints[i].location for i in robot_plan[start_idx:]])
            total_cost += cost
            combined_plan.append(plan)
        return total_cost,combined_plan

    def publish_waypoints(self):
        json_output = json_dumps_dataclass(self.waypoints)
        self._mqtt_client.publish("planner/waypoints", json_output, retain=True)

    def publish_robot_states(self, states :List[RobotState]):
        json_output = json_dumps_dataclass(states)
        self._mqtt_client.publish("planner/robots", json_output, retain=True)


if __name__ == "__main__":
    with open("configuration.json") as f:
        configuration = json.loads(f)
    controller = Controller(configuration)
    controller.main_loop()