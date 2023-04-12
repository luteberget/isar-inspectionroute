
import math
from typing import List, Tuple
from model import Location, RobotParams, RobotState, BatteryConstraint, TaskID, TaskSpec
from model import calculate_distance, calculate_rotation_distance, calculate_along_line, mk_pose
from robotbase import RobotBase, TaskStatus
from datetime import datetime


def vec_len(v: List[float]):
    return math.sqrt(v[0]*v[0] + v[1]*v[1])


def normalize_vec(v: List[float]):
    l = vec_len(v)
    return [x/l for x in v]


class VirtualRobot(RobotBase):
    tasks: List[Tuple[TaskID, TaskSpec, Location]] = []
    events: List[TaskStatus] = []

    current_state: RobotState
    time: datetime
    name: str

    def __init__(self, conf):
        super().__init__()
        self.name = conf["name"]
        self.time = datetime.now()
        self.current_state = RobotState(
            RobotParams(float(conf["speed"]), float(conf["rotation_speed"])),
            Location(conf["init_loc"]["name"], mk_pose(
                conf["init_loc"]["x"], conf["init_loc"]["y"])),
            BatteryConstraint(
                Location(conf["charger"]["name"], mk_pose(
                    conf["charger"]["x"], conf["charger"]["y"])),
                float(conf["battery_distance"]),
                float(conf["remaining_distance"])
            ))

    def set_plan(self, tasks: List[Tuple[TaskID, TaskSpec, Location]]):
        self.tasks = tasks
        self.time = datetime.now()

    def get_state(self):
        self.update_state()
        return self.current_state

    def get_event(self):
        evs = self.events
        self.events = []
        return evs

    def move(self, new_location: Location):
        old_location: Location = self.current_state.current_location  # type: ignore
        dist = calculate_distance(old_location, new_location)
        self.current_state.battery_constraint.remaining_distance -= dist
        self.current_state.current_location = new_location

    def update_state(self):
        dt = (datetime.now() - self.time).total_seconds()
        self.time = datetime.now()

        loc = self.current_state.current_location

        if len(self.tasks) == 0 or loc is None or dt <= 0.0:
            # Nothing is happening
            return

        goal_task_id, goal_task_spec, goal_location = self.tasks[0]
        goal_vector = [goal_location.pose.position.x - loc.pose.position.x,
                       goal_location.pose.position.y - loc.pose.position.y]

        travel_distance = self.current_state.parameters.speed * dt
        if vec_len(goal_vector) <= travel_distance:
            self.move(goal_location)
            self.events.append(TaskStatus(goal_task_id, True))
            self.tasks.pop(0)

            if goal_task_spec == "charge":
                self.current_state.battery_constraint.remaining_distance = self.current_state.battery_constraint.battery_distance

        else:
            # Calculate an intermediate point to move to
            normalized = normalize_vec(goal_vector)
            new_pt = [loc.pose.position.x + travel_distance * normalized[0],
                      loc.pose.position.y + travel_distance * normalized[1]]
            self.move(Location(f"{loc.name.split('->')[0]}->{goal_location.name}",
                               mk_pose(*new_pt)))
