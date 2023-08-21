from dataclasses import dataclass
from typing import List, Tuple
from model import Location, RobotState, TaskSpec, Waypoint, WaypointID, WaypointStatus, calculate_distance
from math import isinf


def greedy_sequence(
    robots: List[RobotState],
    waypoints: List[Waypoint]
) -> List[List[TaskSpec]]:
    print("Planning ", robots)
    """Greedily assign the closest waypoints to the robot with the least travelled distance"""

    remaining_waypoints = [i for i, w in enumerate(
        waypoints) if w.status == WaypointStatus.PENDING]

    @dataclass
    class RobotHypotheticalState:
        location: Location
        time_travelled: float
        remaining_battery_distance: float

    states: List[RobotHypotheticalState] = []
    for r in robots:
        if r.current_location is None:
            raise Exception("Cannot plan with unknown locations")

        states.append(RobotHypotheticalState(r.current_location,
                      0.0, r.battery_constraint.remaining_distance))

    plan: List[List[TaskSpec]] = [[] for _ in robots]
    while len(remaining_waypoints) > 0:
        if all(isinf(s.time_travelled) for s in states):
            print("Warning: could not reach all waypoints")
            break

        # The robot that has travelled the shortest (time or distance) gets its closest waypoint added.
        robot_idxs_by_time = sorted(
            range(len(robots)), key=lambda r: states[r].time_travelled)

        for robot_idx in robot_idxs_by_time:
            # Find the shortest waypoint to the robot

            compatible_waypoints = [p for p in remaining_waypoints if set(
                waypoints[p].capabilities) <= set(robots[robot_idx].parameters.capabilities)]
            
            if len(compatible_waypoints) == 0:
                continue

            selected_wp = min(compatible_waypoints, key=lambda wp: calculate_distance(
                states[robot_idx].location, waypoints[wp].location))

            next_wp_dist = calculate_distance(
                states[robot_idx].location, waypoints[selected_wp].location)
            
            can_charge = robots[robot_idx].battery_constraint.charger_location is not None

            charger_distance = float("inf") if not can_charge  else calculate_distance(
                states[robot_idx].location, robots[robot_idx].battery_constraint.charger_location)  # type: ignore

            next_wp_charger_distance = float("inf") if not can_charge else calculate_distance(
                waypoints[selected_wp].location, robots[robot_idx].battery_constraint.charger_location)  # type: ignore

            # Can it not reach charger?
            if can_charge and charger_distance > states[robot_idx].remaining_battery_distance:
                print("Warning: robot cannot reach charger within battery limits")
                states[robot_idx].time_travelled = float("inf")

            # Does it need to charge before next waypoint?
            elif can_charge and next_wp_dist + next_wp_charger_distance > states[robot_idx].remaining_battery_distance:

                if len(plan[robot_idx]) > 0 and plan[robot_idx][-1] == "charge":
                    print(
                        "Warning: robot cannot reach waypoint with full battery charge")
                    states[robot_idx].time_travelled = float("inf")

                else:

                    plan[robot_idx].append("charge")

                    cl = robots[robot_idx].battery_constraint.charger_location

                    if cl is None:
                        raise Exception("Cannot charge")

                    states[robot_idx] = RobotHypotheticalState(
                        cl,
                        states[robot_idx].time_travelled + charger_distance /
                        robots[robot_idx].parameters.speed,
                        robots[robot_idx].battery_constraint.battery_distance,
                    )

            else:
                # Move waypoint from remaining_waypoint to the plan
                plan[robot_idx].append(WaypointID(selected_wp))
                remaining_waypoints.remove(selected_wp)

                # Update robot's hypothetical state
                states[robot_idx] = RobotHypotheticalState(
                    waypoints[selected_wp].location,
                    states[robot_idx].time_travelled + next_wp_dist /
                    robots[robot_idx].parameters.speed,
                    states[robot_idx].remaining_battery_distance - next_wp_dist,
                )

            break

    return plan
