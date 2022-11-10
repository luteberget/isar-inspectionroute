import math
import json
import argparse
import requests
import time
from requests.compat import urljoin
import paho.mqtt.client as mqtt
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple
from alitra import Pose
import alitra
import robplanpoints
from model import (
    BatteryConstraint,
    Location,
    PlanStep,
    Status,
    calculate_distance,
    json_dumps_dataclass,
)
from greedy import greedy_sequence
from vrp import optimize_waypoint_seq
from set_battery import override_battery_level_topic

all_locations = list(
    map(lambda args: Location(*args), robplanpoints.predefined_poses.items())
)

charger_location = None #all_locations.pop(0)


def loc_string(location: Location):
    "Convert a location JSON message into a short string."
    if location is None:
        return "None"
    return f"Loc({location.name},{location.pose.position.x},{location.pose.position.y},{location.pose.position.z})"


@dataclass
class GreedyPlannerParameters:
    "Parameters for setting up the connections of the greedy planner."
    isar_url: str
    robot_name: str
    mqtt_host: str
    mqtt_port: int
    delay_adding_waypoints: int
    planner_fn: Any
    planner_name: str


def parse_parameters() -> Tuple[GreedyPlannerParameters, List[Location]]:
    "Parse command line arguments for the greedy planner"

    parser = argparse.ArgumentParser(
        description="Simple greedy waypoint planner for ISAR robot."
    )

    parser.add_argument(
        "--isar-url",
        metavar="ISARADDR",
        help="The base URL for the ISAR API.",
        required=True,
    )

    parser.add_argument(
        "--isar-robot-name",
        metavar="NAME",
        help="The robot name used to receive messages from ISAR.",
        default="R2-D2",
    )

    parser.add_argument(
        "--mqtt-hostname",
        metavar="ADDR",
        help="The hostname used for connecting to MQTT.",
        default="localhost",
    )

    parser.add_argument(
        "--mqtt-port",
        metavar="PORT",
        help="The port used for connecting to MQTT.",
        default=1883,
        type=int,
    )

    parser.add_argument(
        "--delay-adding",
        metavar="DELAY",
        help="Add waypoints one by one with this delay.",
        default=10,
        type=int,
    )

    parser.add_argument(
        "--vrp",
        help="Use VRP planner for sequencing and battery constraints.",
        action="store_true",
    )

    args = parser.parse_args()
    print(f"args {args}")

    planner_fn = greedy_sequence
    planner_name = "greedy"
    if args.vrp:
        planner_fn = optimize_waypoint_seq
        planner_name = "vrpy"

    params = GreedyPlannerParameters(
        args.isar_url,
        args.isar_robot_name,
        args.mqtt_hostname,
        args.mqtt_port,
        args.delay_adding,
        planner_fn,
        planner_name,
    )

    return params, all_locations


def convert_pose_isar(location: Location):
    """Convert `Location` to the pose format accepted by the ISAR HTTP interface."""
    return {
        "position": {
            "x": location.pose.position.x,
            "y": location.pose.position.y,
            "z": location.pose.position.z,
            "frame_name": "asset",
        },
        "orientation": {
            "x": location.pose.orientation.x,
            "y": location.pose.orientation.y,
            "z": location.pose.orientation.z,
            "w": location.pose.orientation.w,
            "frame_name": "asset",
        },
        "frame_name": "asset",
    }


def json_to_alitra_pose(json_pose):
    return Pose(
        alitra.Position(
            json_pose["position"]["x"],
            json_pose["position"]["y"],
            json_pose["position"]["z"],
            alitra.Frame(json_pose["position"]["frame"]),
        ),
        alitra.Orientation(
            json_pose["orientation"]["x"],
            json_pose["orientation"]["y"],
            json_pose["orientation"]["z"],
            json_pose["orientation"]["w"],
            alitra.Frame(json_pose["orientation"]["frame"]),
        ),
        frame=alitra.Frame(json_pose["frame"]),
    )


def convert_task_isar(tag, location: Location):
    """Convert a tag string and a `Location` to the task format accepted by the ISAR HTTP interface."""
    return {
        "pose": convert_pose_isar(location),
        "tag": tag,
        "inspection_target": {
            "x": 0,
            "y": 0,
            "z": 0,
            "frame_name": "asset",
        },
        "inspection_types": ["Image"],
    }


def is_task_finished(msg: str):
    return msg == "successful" or msg == "partially_successful" or msg == "failed"


class PlannerState:
    waypoints: List[Tuple[int, Location]] = []
    plan_dirty: bool = False
    robot_is_idle: bool = False
    robot_current_location: Optional[Location] = None
    robot_battery_level: float = 1.0
    params: GreedyPlannerParameters
    last_status_message: Optional[str] = None
    current_mission_id = None
    current_mission_tasks = None
    current_plan = None
    current_wp_seq = None
    _mqtt_client = None

    def __init__(self, params: GreedyPlannerParameters):
        self.params = params
        self._init_mqtt_interface()
        self.publish_waypoints()
        self.publish_plan(0.0, [])

    def _init_mqtt_interface(self):
        mqtt_isar_state_topic = f"isar/{self.params.robot_name}/state"
        mqtt_isar_task_topic = f"isar/{self.params.robot_name}/task"
        mqtt_isar_robot_location_topic = f"isar/{self.params.robot_name}/pose"

        # The callback for when the client receives a CONNACK response from the server.
        def mqtt_connect(client, userdata, flags, rc):
            print("Connected with result code " + str(rc))
            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(mqtt_isar_state_topic)
            client.subscribe(mqtt_isar_task_topic)
            client.subscribe(mqtt_isar_robot_location_topic)
            client.subscribe(override_battery_level_topic)

        # The callback for when a PUBLISH message is received from the server.
        def mqtt_message(client, userdata, msg):
            # print("MQTT: ", msg.topic + " " + str(msg.payload))
            if msg.topic == mqtt_isar_state_topic:
                state_msg = json.loads(msg.payload)
                if state_msg["state"] == "idle":
                    self.robot_is_idle = True
                    self.current_mission_id = None
                    self.current_mission_tasks = None
                else:
                    self.robot_is_idle = False
            elif msg.topic == mqtt_isar_robot_location_topic:
                loc_msg = json.loads(msg.payload)
                # print("LOC", loc_msg)
                self.robot_current_location = Location(
                    "current_location", json_to_alitra_pose(loc_msg["pose"])
                )
            elif msg.topic == override_battery_level_topic:
                batt_msg = json.loads(msg.payload)
                print("Override battery level", batt_msg)
                self.robot_battery_level = batt_msg["level"]
                self.check_plan_battery()
            elif msg.topic == mqtt_isar_task_topic:
                task_msg = json.loads(msg.payload)
                print("Received task message", task_msg)
                if self.current_mission_tasks is not None:
                    for (wp_id, task_id) in self.current_mission_tasks:
                        if task_id == task_msg["task_id"] and is_task_finished(
                            task_msg["status"]
                        ):
                            print(
                                "removing wp_id ",
                                wp_id,
                                "because it has status",
                                task_msg["status"],
                            )
                            self.waypoints = [
                                (id_, wp) for id_, wp in self.waypoints if id_ != wp_id
                            ]
                            self.publish_waypoints()
                    pass
            else:
                print("unhandled message", msg.topic + " " + str(msg.payload))

        client = mqtt.Client()
        client.on_connect = mqtt_connect
        client.on_message = mqtt_message

        client.connect(self.params.mqtt_host, self.params.mqtt_port, 60)

        while not client.is_connected():
            client.loop()

        client.loop_start()

        self._mqtt_client = client

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

    def get_battery_constraint(self) -> BatteryConstraint | None:
        if charger_location is None:
            return None
        return BatteryConstraint(
            charger_location,
            robplanpoints.robot_battery_distance,
            self.robot_battery_level * robplanpoints.robot_battery_distance,
        )

    def plan_and_start_mission(self):
        while True:
            self.cancel_current_mission()

            ok = self.try_plan_and_start_mission()
            if ok:
                break
            else:
                print("Failed to set mission plan. Retrying in 1 sec...")
                time.sleep(1)

    def cancel_current_mission(self):
        # First, cancel existing mission
        if self.current_mission_id is not None or not self.robot_is_idle:
            url = urljoin(self.params.isar_url, "schedule/stop-mission")
            req = requests.post(url)
            print(f"Stopped {req}")
            if req.ok:
                self.current_mission_id = None
                self.current_mission_tasks = None
            else:
                print("Could not stop mission", req, req.json())
                if self.robot_is_idle:
                    self.current_mission_id = None
                    self.current_mission_tasks = None

    def try_plan_and_start_mission(self):
        # If we haven't received the robot's starting location yet
        if self.robot_current_location is None:
            print("Cannot set plan before knowing where the robot is located.")
            return False

        if not self.robot_is_idle:
            print("The robot is not idle, cannot receive misson.")
            return False

        # Sequence the waypoints
        print(self.get_battery_constraint())
        wp_sequence = self.params.planner_fn(
            self.robot_current_location,
            [x for x in self.waypoints],
            self.get_battery_constraint(),
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
        print("write waypoints", json_output);
        self._mqtt_client.publish("planner/waypoints", json_output, retain=True)

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


#
# Initialize the planner state.
#


params, locations_to_add = parse_parameters()
planner = PlannerState(params)

#
# Planner loop
#

previous_added_waypoint_time = time.time()
add_fn = planner.add_waypoint_fn()
while True:
    status = f"Robot status: ready={planner.robot_is_idle} loc={loc_string(planner.robot_current_location)}. "

    if (
        time.time() - previous_added_waypoint_time >= params.delay_adding_waypoints
        and len(locations_to_add) > 0
    ):
        add_fn(locations_to_add.pop(0))
        previous_added_waypoint_time = time.time()

    if not planner.plan_dirty:
        status += "No new information to process."
    else:
        print("Replanning.")
        planner.plan_and_start_mission()
        planner.plan_dirty = False

    planner.set_status(status)
    time.sleep(0.1)
