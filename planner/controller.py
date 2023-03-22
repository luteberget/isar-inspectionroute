import json
import requests
import time

from dataclasses import dataclass
from typing import Any, List, Optional, Tuple
from isarrobot import ISARRobot

from model import (
    BatteryConstraint,
    Location,
    PlanStep,
    Status,
    calculate_distance,
    json_dumps_dataclass,
)
from planner_greedy import greedy_sequence
from robotbase import RobotBase
from planner_vrp import optimize_waypoint_seq

class VirtualRobot:
    pass


class Controller:
    waypoints: List[Tuple[int, Location]] = []
    robots :List[RobotBase] = []
    last_status_message: Optional[str] = None

    def __init__(self, configuration):
        print("Loading configuration:", json.dumps(configuration, indent=2))

        if configuration["planner"] == "greedy":
            self.planner_fn = greedy_sequence
        elif configuration["planner"] == "vrp":
            raise Exception()
        else:
            raise Exception()
        
        for robot_configuration in configuration["robots"]:
            if robot_configuration["type"] == "isar":
                self.robots.append(ISARRobot(robot_configuration))
            elif robot_configuration["type"] == "virtual":
                self.robots.append(VirtualRobot(robot_configuration))

        self.publish_waypoints()
        self.publish_plan(0.0, [])

    def main_loop(self):
        while True:
            time.sleep(0.1)
            robot_states = [robot.get_state() for robot in self.robots]
            if not self.is_plan_valid(robot_states):
                self.update_plan()

    def add_waypoint_fn(self):
        counter = 0

        def f(wp):
            nonlocal counter
            print(f"Received waypoint {loc_string(wp)}.")
            self.waypoints.append((counter, wp))
            self.plan_dirty = True
            self.publish_waypoints()
            counter += 1

        return f


    def plan_and_start_mission(self):
        while True:
            self.cancel_current_mission()

            ok = self.try_plan_and_start_mission()
            if ok:
                break
            else:
                print("Failed to set mission plan. Retrying in 1 sec...")
                time.sleep(1)


    def try_plan_and_start_mission(self):
        # If we haven't received the robot's starting location yet
        if self.robot_current_location is None:
            print("Cannot set plan before knowing where the robot is located.")
            return False

        if not self.robot_is_idle:
            print("The robot is not idle, cannot receive misson.")
            return False

        # Sequence the waypoints
        battery_constraint = self.get_battery_constraint()
        print(battery_constraint)
        if battery_constraint is not None:
            self.publish_battery_constraint(battery_constraint)

        wp_sequence = self.params.planner_fn(
            self.robot_current_location,
            [x for x in self.waypoints],
            battery_constraint,
        )

        # Send the sequence as a mission to ISAR.
        tasks = list(
            map(lambda wp: convert_task_isar(f"wp{wp[0]}", wp[1]), wp_sequence)
        )
        mission = {
            "mission_definition": {
                "tasks": tasks,
            }
        }

        url = urljoin(self.params.isar_url, "schedule/start-mission")
        print(f"Sending ISAR command ({url}) to go to: {mission}")
        req = requests.post(url, json=mission)
        response = req.json()
        print(f"Result: {req} {response}")

        # Store the correspondence between tasks and waypoints, so we
        # can check which waypoints have been successfully visited.
        if req.ok:
            self.current_mission_id = response["id"]
            self.current_mission_tasks = [
                (wp[0], task_data["id"])
                for task_data, wp in zip(response["tasks"], wp_sequence)
            ]
            print(
                "Current mission",
                self.current_mission_id,
                "tasks",
                self.current_mission_tasks,
            )
            self.robot_is_idle = False

            # Send the current plan over MQTT
            self.current_wp_seq = wp_sequence
            cost, self.current_plan = self.integrate_plan(wp_sequence)
            self.publish_plan(cost, self.current_plan)

            return True
        else:
            print("Warning: ISAR command failed", response)
            return False

    def set_status(self, s):
        if self.last_status_message == s:
            return
        self.last_status_message = s
        print("Status: ", s)

    def publish_plan(self, cost: float, plan: List[PlanStep]):
        planner = self.params.planner_name if len(plan) > 0 else "none"
        status = Status(planner, cost, plan)
        json_output = json_dumps_dataclass(status)
        self._mqtt_client.publish("planner/status", json_output, retain=True)

    def publish_waypoints(self):
        json_output = json_dumps_dataclass([ x for _,x in self.waypoints])
        print("write waypoints", json_output)
        self._mqtt_client.publish("planner/waypoints", json_output, retain=True)

    def publish_battery_constraint(self, battery_constraint :BatteryConstraint):
        json_output = json_dumps_dataclass(battery_constraint)
        print("write battery constraint", json_output);
        self._mqtt_client.publish("planner/battery_constraint", json_output, retain=True)

    def integrate_plan(self, wp_sequence: List[Tuple[int, Location]]):
        cost = 0
        battery = self.robot_battery_level * robplanpoints.robot_battery_distance
        prev_loc = self.robot_current_location
        plan = []

        for x, loc in wp_sequence:
            d = calculate_distance(prev_loc, loc)
            cost += d
            battery -= d
            if battery < 0.05:
                print(
                    "Warning: less than 5 percent battery",
                    battery,
                    "at",
                    loc_string(loc),
                )
            plan.append(PlanStep(loc, battery))
            if x == "charger":
                battery = robplanpoints.robot_battery_distance
            prev_loc = loc
        return cost, plan

    def check_plan_battery(self):
        if self.current_wp_seq is None:
            return

        _cost, plan = self.integrate_plan(self.current_wp_seq)
        if min([x.remaining_battery for x in plan], default=1.0) < 0.0:
            print(
                "Battery level changed and the plan is not battery positive. Replanning."
            )
            self.plan_dirty = True


if __name__ == "__main__":
    with open("configuration.json") as f:
        configuration = json.loads(f)
    controller = Controller(configuration)
    controller.main_loop()