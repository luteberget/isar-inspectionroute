import rospy
import json
from geometry_msgs.msg import PoseStamped


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


counter = 0
def convert(msg):
    global counter
    counter += 1
    print(json.dumps({"name": f"pt{counter}", "pose": convert_pose_isar(msg) }))

rospy.init_node("rospose2json", anonymous=True)
rospy.Subscriber("/rospose2json", PoseStamped, convert)

while True:
    if rospy.is_shutdown():
        raise Exception("ROS exited.")
    rospy.sleep(1)
