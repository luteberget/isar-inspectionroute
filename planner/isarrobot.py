import requests
from requests.compat import urljoin
import paho.mqtt.client as mqtt
from alitra import Pose
import alitra

import json
from typing import Optional

from model import Location
from robotbase import RobotBase


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

   
   
class ISARRobot(RobotBase):
    robot_is_idle: bool = False
    robot_current_location: Optional[Location] = None
    robot_battery_level: float = 1.0
    
    current_mission_id = None
    current_mission_tasks = None
    current_plan = None
    current_wp_seq = None
    _mqtt_client = None

    def get_battery_constraint(self) -> BatteryConstraint | None:
        if charger_location is None:
            return None
        return BatteryConstraint(
            charger_location,
            robplanpoints.robot_battery_distance,
            self.robot_battery_level * robplanpoints.robot_battery_distance,
        )


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
