from enum import Enum
from typing import List
from alitra import Pose
from dataclasses import dataclass
import math
import dataclasses, json

WaypointID = int

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

WaypointStatus = Enum("WaypointStatus", ["PENDING","SUCCESS","FAILURE"])

@dataclass
class Waypoint:
    status: WaypointStatus
    is_charger :bool
    location :Location


@dataclass
class BatteryConstraint:
    charger_location: Location
    battery_distance: float
    remaining_distance: float

@dataclass
class RobotState:
    current_location :Location
    battery_constraint :BatteryConstraint | None


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
    o1 = wp1.pose.orientation
    o2 = wp2.pose.orientation
    dox = o2.x - o1.x
    doy = o2.y - o1.x
    doz = o2.z - o1.z
    dow = o2.w - o1.w
    wps = [wp1]*steps
    for ii in range(steps):
        wps[ii].pose.position.x += (ii+1)*dx/steps 
        wps[ii].pose.position.y += (ii+1)*dy/steps
        wps[ii].pose.position.z += (ii+1)*dz/steps
        wps[ii].pose.orientation.x += (ii+1)*dox/steps
        wps[ii].pose.orientation.y += (ii+1)*doy/steps
        wps[ii].pose.orientation.z += (ii+1)*doz/steps
        wps[ii].pose.orientation.w += (ii+1)*dow/steps
        if wps[ii].pose == wp2.pose:
            wps[ii].name = wp2.name
        else:
            wps[ii].name = wp1.name + "->" + wp2.name
    return wps

@dataclass
class PlanStep:
    location: Location
    remaining_battery: float


@dataclass
class Status:
    solver: str
    total_cost: float
    plan: List[List[PlanStep]]


def loc_string(location: Location):
    "Convert a location JSON message into a short string."
    if location is None:
        return "None"
    return f"Loc({location.name},{location.pose.position.x},{location.pose.position.y},{location.pose.position.z})"


def integrate_robot_plan(state :RobotState, wp_sequence: List[Location]):
        cost = 0
        battery = 1.0
        if state.battery_constraint is not None:
            battery = state.battery_constraint.remaining_distance
        prev_loc = state.current_location
        plan = []

        for loc in wp_sequence:
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
            if loc.is_charger:
                battery = state.battery_constraint.battery_distance
            prev_loc = loc
        return cost, plan
