import math
import json
import argparse
import rospy
import requests
from requests.compat import urljoin
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import paho.mqtt.client as mqtt
from dataclasses import dataclass
from typing import List, Optional, Tuple


def loc_string(location):
    "Convert a PoseStamped ROS message into a short string."
    if location is None:
        return "None"
    return f"Loc({location.pose.position.x},{location.pose.position.y},{location.pose.position.z})"


@dataclass
class GreedyPlannerParameters:
    "Parameters for setting up the connections of the greedy planner."
    isar_url: str
    waypoints_address: str
    start_address: str
    robot_name: str
    mqtt_host: str
    mqtt_port: int


def parse_parameters() -> GreedyPlannerParameters:
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
        default="R2-D2"
    )

    parser.add_argument(
        "--mqtt-hostname",
        metavar="ADDR",
        help="The hostname used for connecting to MQTT.",
        default="localhost"
    )

    parser.add_argument(
        "--mqtt-port",
        metavar="PORT",
        help="The port used for connecting to MQTT.",
        default=1883
    )

    parser.add_argument("--start-addr",
                        metavar="SA",
                        help="ROS message path for start message. If this value is set, \n" +
                        "the planner will wait for any message on this address before \n" +
                        "starting the planning and execution.")

    parser.add_argument("--waypoints-addr",
                        metavar="WA",
                        default="/greedy_planner/add_waypoint",
                        help="ROS message path for adding waypoints to the planner. \n" +
                        "Defaults to /greedy_planner/add_waypoint")

    args = parser.parse_args()
    print(f"args {args}")

    return GreedyPlannerParameters(
        args.isar_url,
        args.waypoints_addr,
        args.start_addr,
        args.isar_robot_name,
        args.mqtt_hostname,
        args.mqtt_port)


def calculate_distance(wp1: PoseStamped, wp2: PoseStamped) -> float:
    """Distance matrix for waypoints, here simplified to Euclidean distance."""
    p1 = wp1.pose.position
    p2 = wp1.pose.position
    dx = p2.x-p1.x
    dy = p2.y-p1.y
    dz = p2.z-p1.z
    return math.sqrt(dx*dx+dy*dy+dz*dz)


def greedy_sequence(loc: PoseStamped, wps: List[Tuple[int, PoseStamped]]) -> List[Tuple[int, PoseStamped]]:
    """Sequence the waypoints, starting from the initial location `loc`.
    If the initial location is not given, the first waypoint is used as the current location.
    Returns a permutation of `wps` by greedily selecting the closest point from the previous location."""

    xs = []
    if loc is None and len(wps) > 0:
        xs.append(wps.pop(0))
        loc = xs[-1][1]

    while len(wps) > 0:
        next_location = min(wps, key=lambda wp: calculate_distance(loc, wp[1]))
        wps.remove(next_location)
        xs.append(next_location)
        loc = xs[-1][1]

    print("Greedy sequence ", xs)
    return xs


def convert_pose_isar(location: PoseStamped):
    """Convert `PoseStamped` to the pose format accepted by the ISAR HTTP interface."""
    return {
        "position": {
            "x": location.pose.position.x,
            "y": location.pose.position.y,
            "z": location.pose.position.z,
            "frame_name": "asset"
        },
        "orientation": {
            "x": location.pose.orientation.x,
            "y": location.pose.orientation.y,
            "z": location.pose.orientation.z,
            "w": location.pose.orientation.z,
            "frame_name": "asset"
        },
        "frame_name": "asset"
    }


def convert_task_isar(tag, location: PoseStamped):
    """Convert a tag string and a `PoseStamped` to the task format accepted by the ISAR HTTP interface."""
    return {
        "pose": convert_pose_isar(location),
        "tag": tag,
        "inspection_target": {
            "x": location.pose.position.x,
            "y": location.pose.position.y,
            "z": location.pose.position.z,
            "frame_name": "asset"
        },
        "inspection_types": ["Image"],
    }


class PlannerState:
    planner_running: bool = False
    waypoints: List[Tuple[int, PoseStamped]] = []
    waypoints_dirty :bool = False
    robot_is_ready: bool = False
    robot_current_location: Optional[PoseStamped] = None
    params: GreedyPlannerParameters
    last_status_message: Optional[str] = None
    current_mission_id = None
    current_mission_tasks = None

    def __init__(self, params: GreedyPlannerParameters):
        self.params = params
        self._init_ros_interface()
        self._init_mqtt_interface()

    def _init_ros_interface(self):
        "Initialize ROS interface."
        rospy.init_node('greedy_planner', anonymous=True)
        rospy.Subscriber(self.params.waypoints_address,
                         PoseStamped, self.add_waypoint_fn())

        # Update the planner_running variable if we need to wait for the start signal.
        if self.params.start_address is not None:
            rospy.Subscriber(self.params.start_address,
                             String, self.start_planner_fn())
        else:
            self.start_planner_fn()()

    def _init_mqtt_interface(self):
        mqtt_isar_state_topic = f"isar/{self.params.robot_name}/state"
        mqtt_isar_task_topic = f"isar/{self.params.robot_name}/task"
        mqtt_isar_robot_location_topic = f"isar/{self.params.robot_name}/pose"

        # The ncallback for when the client receives a CONNACK response from the server.
        def mqtt_connect(client, userdata, flags, rc):
            print("Connected with result code "+str(rc))
            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(mqtt_isar_state_topic)

        # The callback for when a PUBLISH message is received from the server.
        def mqtt_message(client, userdata, msg):
            print("MQTT: ", msg.topic+" "+str(msg.payload))
            global robot_is_ready
            if msg.topic == mqtt_isar_state_topic:
                state_msg = json.loads(msg.payload)
                self.robot_is_ready = state_msg["state"] == "idle"
            elif msg.topic == mqtt_isar_robot_location_topic:
                loc_msg = json.loads(msg.payload)
                print("LOC", loc_msg)
                self.robot_current_location = loc_msg["pose"]
            elif msg.topic == mqtt_isar_task_topic:
                task_msg = json.loads(msg.payload)
                print("Received task message", task_msg)
                if self.current_mission_tasks is not None:
                    for (wp_id, task_id) in self.current_mission_tasks:
                        if task_id == task_msg["id"] and task_msg["state"] == "success":
                            print("removing wp_id ", wp_id)
                            self.waypoints = [
                                (id_, wp) for id_, wp in self.waypoints if id_ != wp_id]
                    pass
            else:
                print("unhandled message", msg.topic+" "+str(msg.payload))

        client = mqtt.Client()
        client.on_connect = mqtt_connect
        client.on_message = mqtt_message

        client.loop_start()
        client.connect(self.params.mqtt_host, self.params.mqtt_port, 60)

    def start_planner_fn(self):
        def f():
            self.planner_running = True
        return f

    def add_waypoint_fn(self):
        counter = 0

        def f(wp):
            nonlocal counter
            print(f"Received waypoint {loc_string(wp)}.")
            self.waypoints.append((counter, wp))
            self.waypoints_dirty = True
            counter += 1
        return f

    def plan_and_start_mission(self):
        # First, cancel existing mission
        if self.current_mission_id is not None:
            url = urljoin(self.params.isar_url, "schedule/stop-mission")
            req = requests.post(url)
            print(f"Stopped {req}")

            self.current_mission_id = None
            self.current_mission_tasks = None

        # Sequence the waypoints
        wp_sequence = greedy_sequence(self.robot_current_location, [x for x in self.waypoints])

        # Send the sequence as a mission to ISAR.
        tasks = list(map(lambda wp: convert_task_isar(
            f"wp{wp[0]}", wp[1]), wp_sequence))
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
        self.current_mission_id = response["id"]
        self.current_mission_tasks = [
            (wp[0], task_data["id"])
            for task_data, wp in zip(response["tasks"], wp_sequence)
        ]
        self.robot_is_ready = False

    def set_status(self, s):
        if self.last_status_message == s:
            return
        self.last_status_message = s
        print("Status: ", s)

    def main_loop_step(self):
        # ROS interface sleeps.
        # MQTT interface runs on background thread.

        if rospy.is_shutdown():
            raise Exception("ROS exited.")
        rospy.sleep(1)

#
# Initialize the planner state.
#


params = parse_parameters()
planner = PlannerState(params)

#
# Planner loop
#

while True:
    status = f"Robot status: ready={planner.robot_is_ready} loc={loc_string(planner.robot_current_location)}. "

    if planner.planner_running:
        if not planner.waypoints_dirty:
            status += "No new waypoints to process."
        else:
            print("Replanning.")
            planner.plan_and_start_mission()
            planner.waypoints_dirty = False
    else:
        status += f"Planner waiting for start message."

    planner.set_status(status)
    planner.main_loop_step()
