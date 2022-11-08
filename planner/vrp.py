from typing import List, Tuple
from networkx import DiGraph

from model import BatteryConstraint, Location, calculate_distance
from vrpy import VehicleRoutingProblem

SOLVER_TIME_LIMIT: float = 15


def optimize_waypoint_seq(
    start_location: Location,
    wps: List[Tuple[int, Location]],
    battery: BatteryConstraint | None,
) -> List[Tuple[int, Location]]:
    print(wps)
    if battery is None:
        return seq_open(start_location, wps)
    else:
        return seq_charging(start_location, wps, battery)


def seq_open(
    start_location: Location,
    wps: List[Tuple[int, Location]],
) -> List[Tuple[int, Location]]:

    G = DiGraph()

    # Add edge from start location (Source) to all
    for i in range(len(wps)):
        d = calculate_distance(start_location, wps[i][1])
        G.add_edge("Source", i, cost=d)

    # Add wp edges
    for i in range(len(wps)):
        for j in range(i + 1, len(wps)):
            d = calculate_distance(wps[i][1], wps[j][1])
            print("dist", i, j, d)
            G.add_edge(i, j, cost=d)
            G.add_edge(j, i, cost=d)

    # Add sink edges
    for i in range(len(wps)):
        G.add_edge(i, "Sink", cost=0)

    prob = VehicleRoutingProblem(G, num_vehicles=1)
    prob.solve(time_limit=SOLVER_TIME_LIMIT)
    print("Open VRP solution", prob.best_routes_cost, prob.best_routes)
    if len(prob.best_routes) > 1:
        print("Warning: multiple routes found by the Open VRP")

    for route in prob.best_routes.values():
        return [wps[i] for i in route[1:-1]]


def seq_charging(
    start_location: Location,
    wps: List[Tuple[int, Location]],
    battery: BatteryConstraint,
) -> List[Tuple[int, Location]]:

    G = DiGraph()

    # Add a zero-length edge from source to start_location
    G.add_edge(
        "Source", -1, cost=0, time=battery.battery_distance - battery.remaining_distance
    )

    # Add edges from source and from start_location to all wps
    for a_name, a_loc in [("Source", battery.charger_location), (-1, start_location)]:
        for b_name, (_, b_loc) in enumerate(wps):
            d = calculate_distance(a_loc, b_loc)
            G.add_edge(a_name, b_name, cost=d, time=d)

    # Add wp edges
    for i in range(len(wps)):
        for j in range(i + 1, len(wps)):
            d = calculate_distance(wps[i][1], wps[j][1])
            G.add_edge(i, j, cost=d, time=d)
            G.add_edge(j, i, cost=d, time=d)

    # Return to charger (sink)
    for name, (_, loc) in enumerate(wps) + [(-1, (None, start_location))]:
        d = calculate_distance(loc, battery.charger_location)
        G.add_edge(name, "Sink", cost=d, time=d)

    prob = VehicleRoutingProblem(G, duration=battery.battery_distance)
    prob.solve(time_limit=SOLVER_TIME_LIMIT)
    print(prob.best_routes)

    routes = prob.best_routes.copy()

    initial_routes = [key for key, r in routes.items() if len(r) >= 1 and r[0] == -1]

    if len(initial_routes) != 1:
        print("Warning: Charging VRP did not find a unique initial route.")

    output_route = []

    def add_output_route(route):
        for node in route[1:-1]:
            if node == -1:
                continue
            output_route.append(wps[node])
        output_route.append(("charger", battery.charger_location))

    for initial_route_key in initial_routes:
        add_output_route(routes.pop(initial_route_key))

    for route in routes.values():
        add_output_route(route)

    return output_route


def test1():
    G = DiGraph()
    G.add_edge("Source", 1, cost=1)
    G.add_edge("Source", 2, cost=2)
    G.add_edge(1, "Sink", cost=0)
    G.add_edge(2, "Sink", cost=2)
    G.add_edge(1, 2, cost=1)
    G.add_edge(2, 1, cost=1)
    # for v in G.nodes():
    #    if v not in ["Source", "Sink"]:
    #       G.nodes[v]["demand"] = 5

    prob = VehicleRoutingProblem(G, duration=10, load_capacity=99)
    prob.solve()
    print(prob.best_routes)
    print(prob.best_routes_load)


if __name__ == "__main__":
    test1()
