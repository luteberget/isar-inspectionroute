
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

from model import Location, RobotState, WaypointID

@dataclass
class TaskStatus:
    waypoint :int
    success :bool

class RobotBase(ABC):

    @abstractmethod
    def set_plan(waypoints :List[Tuple[WaypointID, Location]]):
        pass

    @abstractmethod
    def get_state(self) -> RobotState:
        pass

    @abstractmethod
    def get_event(self) -> List[TaskStatus]:
        pass