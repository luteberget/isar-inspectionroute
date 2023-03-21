
from abc import ABC, abstractmethod
from typing import List, Tuple

from model import Location, RobotState

class RobotBase(ABC):

    @abstractmethod
    def set_plan(waypoints :List[Tuple[int, Location]]):
        pass

    @abstractmethod
    def get_state(self) -> RobotState:
        pass