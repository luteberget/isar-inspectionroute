from typing import List, Tuple
from model import BatteryConstraint, Location, calculate_distance

def greedy_sequence(
    loc: List[Tuple[Location, BatteryConstraint]], wps: List[Tuple[int, Location]],
) -> List[List[Tuple[int, Location]]]:
    
    # TODO extend to multi-robot 

    # """Sequence the waypoints, starting from the initial location `loc`.
    # If the initial location is not given, the first waypoint is used as the current location.
    # Returns a permutation of `wps` by greedily selecting the closest point from the previous location."""

    # # NOTE battery constraint is ignored

    # xs = []
    # if loc is None and len(wps) > 0:
    #     xs.append(wps.pop(0))
    #     loc = xs[-1][1]

    # while len(wps) > 0:
    #     next_location = min(wps, key=lambda wp: calculate_distance(loc, wp[1]))
    #     wps.remove(next_location)
    #     xs.append(next_location)
    #     loc = xs[-1][1]

    # print("Greedy sequence ", xs)
    # return xs
