import paho.mqtt.publish as mqtt
import json
import sys

override_battery_level_topic = "planner/override_battery_level"

if __name__ == "__main__":
    level = float(sys.argv[1])
    mqtt.single(override_battery_level_topic, json.dumps({"level": level}))
