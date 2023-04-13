import paho.mqtt.publish as mqtt
import json
import sys
from model import mqtt_planner_add_waypoint_topic


points_2d = [
    [1, 1],
    [1, 2],
    [1, 3],
    [2, 1],
    [2, 2],
    [2, 3],
    [3, 1],
    [3, 2],
    [3, 3],
]


def load_default_waypoints():
    for idx, pt_2d in enumerate(points_2d):
        mqtt.single(
            mqtt_planner_add_waypoint_topic,
            json.dumps({"name": f"pt{idx}", "x": pt_2d[0], "y": pt_2d[1], "z": 0.0, "capabilities": []}),
        )


if __name__ == "__main__":
    load_default_waypoints()
