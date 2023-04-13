from enum import Enum
from typing import List, Literal
from alitra import Pose, Position, Orientation
from dataclasses import dataclass
import math
import dataclasses
import json

mqtt_planner_add_waypoint_topic: str = "planner/add_waypoint"
mqtt_planner_planning_enabled_topic: str = "planner/enable_planning"


WaypointID = int
TaskID = int


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


class WaypointStatus(str, Enum):
    PENDING = "Pending"
    SUCCESS = "Success"
    FAILURE = "Failure"


@dataclass
class Waypoint:
    status: WaypointStatus
    is_charger: bool
    location: Location


@dataclass
class BatteryConstraint:
    charger_location: Location | None
    battery_distance: float
    remaining_distance: float


@dataclass
class RobotParams:
    speed: float
    rotation_speed: float


@dataclass
class RobotState:
    parameters: RobotParams
    current_location: Location | None
    battery_constraint: BatteryConstraint


@dataclass
class PlanStep:
    location: Location
    time: float
    remaining_battery: float


@dataclass
class PlanStatus:
    status_msg :str
    solver: str
    total_cost: float
    robot_plans: List[List[PlanStep]]
    plan_version: int
    plan_reason: str | None


@dataclass
class ControllerStatus:
    waypoints: List[Waypoint]
    robots: List[RobotState]
    plan: PlanStatus


TaskSpec = Literal["charge"] | WaypointID


def mk_pose(x, y, z=0.0) -> Pose:
    return Pose(
        position=Position(x, y, z, frame=None),  # type: ignore
        orientation=Orientation(0, 0, 0, 1, frame=None),  # type: ignore
        frame=None,  # type: ignore
    )


def calculate_distance(wp1: Location, wp2: Location) -> float:
    """Distance matrix for waypoints, here simplified to Euclidean distance."""
    p1 = wp1.pose.position
    p2 = wp2.pose.position
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    dz = p2.z - p1.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def calculate_rotation_distance(wp1: Location, wp2: Location) -> float:
    """Distance matrix for waypoint orientation, here simplified to Euclidean distance."""
    o1 = wp1.pose.orientation
    o2 = wp2.pose.orientation
    return math.sqrt(
        (o2.x - o1.x) ** 2
        + (o2.y - o1.y) ** 2
        + (o2.z - o1.z) ** 2
        + (o2.w - o1.w) ** 2
    )


def calculate_along_line(
    wp1: Location, wp2: Location, position_dist: float, orientation_dist: float
) -> Location:
    """Finds wp3 along the line between wp1 and wp2 with a given distance from wp1"""
    wp3 = wp1
    p1 = wp1.pose.position
    p2 = wp2.pose.position
    dist = calculate_distance(
        wp1, wp2) if calculate_distance(wp1, wp2) != 0 else 1
    wp3.pose.position.x += position_dist * (p2.x - p1.x) / dist
    wp3.pose.position.y += position_dist * (p2.y - p1.y) / dist
    wp3.pose.position.z += position_dist * (p2.z - p1.z) / dist
    o1 = wp1.pose.orientation
    o2 = wp2.pose.orientation
    dist = (
        calculate_rotation_distance(wp1, wp2)
        if calculate_rotation_distance(wp1, wp2) != 0
        else 1
    )
    wp3.pose.orientation.x += orientation_dist * (o2.x - o1.x) / dist
    wp3.pose.orientation.y += orientation_dist * (o2.y - o1.y) / dist
    wp3.pose.orientation.z += orientation_dist * (o2.z - o1.z) / dist
    wp3.pose.orientation.w += orientation_dist * (o2.w - o1.w) / dist
    if wp3.pose == wp2.pose:
        wp3.name = wp2.name
    else:
        wp3.name = wp1.name.split("->")[0] + "->" + wp2.name
    return wp3


def loc_string(location: Location):
    "Convert a location JSON message into a short string."
    if location is None:
        return "None"
    return f"Loc({location.name},{location.pose.position.x},{location.pose.position.y},{location.pose.position.z})"


def integrate_robot_plan(state: RobotState, waypoints: List[Waypoint], tasks: List[TaskSpec]):
    if state.current_location is None:
        raise Exception("Cannot integrate robot plan with unknown position.")

    cost = 0
    time = 0
    battery = state.battery_constraint.remaining_distance
    prev_loc = state.current_location
    plan = []

    for task in tasks:
        location = state.battery_constraint.charger_location if task == "charge" else waypoints[
            task].location
        if location is None:
            raise Exception("Location error")
        d = calculate_distance(prev_loc, location)
        cost += d
        battery -= d
        if battery < 0.05:
            print(
                "Warning: less than 5 percent battery",
                battery,
                "at",
                loc_string(location),
            )
        time += d/state.parameters.speed
        plan.append(PlanStep(location, time, battery))
        if task == "charge":
            battery = state.battery_constraint.battery_distance
            plan.append(PlanStep(location, time, battery))
        prev_loc = location
    return cost, plan
