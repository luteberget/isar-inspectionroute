from typing import List
from alitra import Pose
from dataclasses import dataclass
import math
import dataclasses, json


class DataclassJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if dataclasses.is_dataclass(o):
            return dataclasses.asdict(o)
        return super().default(o)


def json_dumps_dataclass(foo):
    return json.dumps(foo, cls=DataclassJSONEncoder)


@dataclass
class Location:
    name: str
    pose: Pose



@dataclass
class BatteryConstraint:
    charger_location: Location
    battery_distance: float
    remaining_distance: float

@dataclass
class RobotState:
    current_location :Location
    battery_constraint :BatteryConstraint


def calculate_distance(wp1: Location, wp2: Location) -> float:
    """Distance matrix for waypoints, here simplified to Euclidean distance."""
    p1 = wp1.pose.position
    p2 = wp2.pose.position
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    dz = p2.z - p1.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)

def calculate_line(wp1: Location,wp2: Location,steps: int) -> List[Location]:
    p1 = wp1.pose.position
    p2 = wp2.pose.position
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    dz = p2.z - p1.z
    wps = [wp1]*steps
    for ii in range(steps):
        wps[ii].x += dx/steps 
        wps[ii].y += dy/steps
        wps[ii].z += dz/steps
    return wps

@dataclass
class PlanStep:
    location: Location
    remaining_battery: float


@dataclass
class Status:
    solver: str
    total_cost: float
    plan: List[PlanStep]



def loc_string(location: Location):
    "Convert a location JSON message into a short string."
    if location is None:
        return "None"
    return f"Loc({location.name},{location.pose.position.x},{location.pose.position.y},{location.pose.position.z})"
