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


def calculate_distance(wp1: Location, wp2: Location) -> float:
    """Distance matrix for waypoints, here simplified to Euclidean distance."""
    p1 = wp1.pose.position
    p2 = wp2.pose.position
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    dz = p2.z - p1.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


@dataclass
class PlanStep:
    location: Location
    remaining_battery: float


@dataclass
class Status:
    solver: str
    total_cost: float
    plan: List[PlanStep]
