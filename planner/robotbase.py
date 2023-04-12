
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

from model import Location, RobotState, TaskID, TaskSpec, WaypointID

@dataclass
class TaskStatus:
    task_id :int
    success :bool

class RobotBase(ABC):
    @abstractmethod
    def set_plan(self, waypoints :List[Tuple[TaskID, TaskSpec, Location]]):
        pass

    @abstractmethod
    def get_state(self) -> RobotState:
        pass

    @abstractmethod
    def get_event(self) -> List[TaskStatus]:
        pass