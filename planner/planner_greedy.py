from typing import List, Tuple
from model import RobotState, Waypoint, WaypointStatus, calculate_distance

def greedy_sequence(
    robots: List[RobotState], 
    waypoints :List[Waypoint]
) -> List[List[int]]:
    """Greedily assign the closest waypoints to the robot with the least travelled distance"""
    # NOTE: Does not respect the battery constraint.

    remaining_waypoints = [i for i,w in enumerate(waypoints) if w.status == WaypointStatus.PENDING]

    plan = [[] for _ in robots]
    robot_locs = [r.current_location for r in robots]
    robot_travel = [0 for _ in robots]

    # The robot that has travelled the shortest gets its closest waypoint added.

    while len(remaining_waypoints) > 0:

        # Find the robot that has travelled the shortest
        robot = min(range(len(robots)), key=lambda r: robot_travel[r])

        # Find the shortest waypoint to the robot
        selected_wp = min( remaining_waypoints, key=lambda wp: calculate_distance(robot_locs[robot], waypoints[wp].location))

        # Move waypoint from remaining_waypoint to the plan
        plan[robot].append(selected_wp)
        remaining_waypoints.remove(selected_wp)

        # Move robot's hypothetical location
        robot_travel += calculate_distance(robot_locs[robot], waypoints[selected_wp].location)
        robot_locs[robot] = waypoints[selected_wp].location


    return plan
