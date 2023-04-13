import json
import time
import paho.mqtt.client as mqtt
from typing import List, Optional
from isarrobot import ISARRobot

# from isarrobot import ISARRobot
from load_waypoints import load_default_waypoints

from model import (
    ControllerStatus,
    Location,
    PlanStatus,
    RobotState,
    TaskSpec,
    Waypoint,
    WaypointID,
    WaypointStatus,
    integrate_robot_plan,
    json_dumps_dataclass,
    mk_pose,
    mqtt_planner_add_waypoint_topic,
    mqtt_planner_planning_enabled_topic
)

from planner_greedy import greedy_sequence
from robotbase import RobotBase
from virtualrobot import VirtualRobot


class Controller:
    waypoints: List[Waypoint] = []
    robots: List[RobotBase] = []
    last_status_message: Optional[str] = None
    current_plan_progress: List[int] = []
    plan_forced_dirty = True
    current_plan: List[List[TaskSpec]] = []
    current_plan_version: int = 0
    current_plan_reason: str | None = None
    use_planning_enabled_message: bool
    planning_enabled: bool

    def __init__(self, configuration):
        print("Loading configuration:", json.dumps(configuration, indent=2))
        self.planner_name = configuration["planner"]
        if configuration["planner"] == "greedy":
            self.planner_fn = greedy_sequence
        elif configuration["planner"] == "vrp":
            raise Exception("not implemented")
        else:
            raise Exception("unknown planner")

        for robot_configuration in configuration["robots"]:
            if robot_configuration["type"] == "isar":
                self.robots.append(ISARRobot(robot_configuration))
            elif robot_configuration["type"] == "virtual":
                self.robots.append(VirtualRobot(robot_configuration))
            else:
                raise Exception("Unknown robot type")

            self.current_plan.append([])
            self.current_plan_progress.append(0)

        self.use_planning_enabled_message = configuration.get(
            "use-planning-enabled-message", False)
        self.planning_enabled = not self.use_planning_enabled_message

        self._init_mqtt(configuration)
        self.publish_state([])

    def _init_mqtt(self, configuration):
        hostname = configuration["mqtt-hostname"]
        port = configuration["mqtt-port"] or 1883

        def mqtt_connect(client, userdata, flags, rc):
            print("Controller connected to MQTT with result code " + str(rc))
            # Subscribing in on_connect() means that if we lose the connection and
            # reconnect then subscriptions will be renewed.
            client.subscribe(mqtt_planner_add_waypoint_topic)
            client.subscribe(mqtt_planner_planning_enabled_topic)

        def mqtt_message(client, userdata, msg):
            if msg.topic == mqtt_planner_planning_enabled_topic:
                if self.use_planning_enabled_message:
                    self.planning_enabled = msg.payload.decode(
                        "utf-8") == "true"
                    if self.planning_enabled:
                        self.plan_forced_dirty = True

            elif msg.topic == mqtt_planner_add_waypoint_topic:
                state_msg = json.loads(msg.payload)
                msg_ok = (
                    "name" in state_msg
                    and "x" in state_msg
                    and "y" in state_msg
                    and "z" in state_msg
                    and "capabilities" in state_msg
                )
                self.log_msg(f"Received new waypoint: {msg.payload}")

                if not msg_ok:
                    self.log_msg("Warning: parsing waypoint failed")
                else:
                    loc = Location(
                        state_msg["name"],
                        mk_pose(state_msg["x"],
                                state_msg["y"], state_msg["z"]),
                    )
                    self.waypoints.append(
                        Waypoint(WaypointStatus.PENDING, False, loc, state_msg["capabilities"]),
                    )

        client = mqtt.Client()
        client.on_connect = mqtt_connect
        client.on_message = mqtt_message
        client.connect(hostname, port, 60)
        while not client.is_connected():
            client.loop()
        client.loop_start()
        self._mqtt_client = client

    def robot_ready(self, state: RobotState):
        return state.current_location is not None

    def main_loop(self):

        while True:
            robot_states = [robot.get_state() for robot in self.robots]
            self.updates_from_robot_events()

            robots_ready = all(self.robot_ready(s) for s in robot_states)

            if not robots_ready:
                self.set_status("Waiting for robots to be ready.")
            elif not self.planning_enabled:
                self.set_status("Planning is disabled")
            else:
                self.set_status("Planning enabled.")
                plan_flaw = self.is_plan_valid(robot_states)
                if plan_flaw is not None:
                    self.log_msg("Updating plan because of flaw: " + plan_flaw)
                    self.update_plan(robot_states, plan_flaw)

            self.publish_state(robot_states)
            time.sleep(0.5)

    def updates_from_robot_events(self):
        for robot_idx, robot in enumerate(self.robots):
            for task_status in robot.get_event():
                robot_plan = self.current_plan[robot_idx]
                task_spec = robot_plan[task_status.task_id]

                if task_spec != "charge" and task_spec >= len(self.waypoints):
                    self.log_msg(
                        f"WARNING: unknown waypoint id {task_spec}")
                    continue

                    # Mark the waypoint in the waypoint list.
                if task_status.success:
                    self.log_msg(
                        f"Successfully completed task {task_status} => {task_spec} with robot {robot}"
                    )
                    if task_spec != "charge":
                        self.waypoints[task_spec].status = WaypointStatus.SUCCESS
                else:
                    self.log_msg(
                        f"Failed to complete task {task_status} => {task_spec} with robot {robot}")
                    if task_spec != "charge":
                        self.waypoints[task_spec].status = WaypointStatus.FAILURE

                plan_idx = robot_plan.index(task_spec)
                progress = self.current_plan_progress[robot_idx]
                if plan_idx != progress:
                    self.log_msg(
                        f"Warning: robot finished task that was not the first planned."
                    )
                    robot_plan[plan_idx], robot_plan[progress] = (
                        robot_plan[progress],
                        robot_plan[plan_idx],
                    )

                self.current_plan_progress[robot_idx] += 1

    def log_msg(self, msg):
        self._mqtt_client.publish("planner/message", msg, retain=False)
        print(msg)

    def is_plan_valid(self, robot_states: List[RobotState]):

        if self.plan_forced_dirty:
            self.plan_forced_dirty = False
            return "First plan."

        pending_waypoints = frozenset(
            (
                i
                for i, wp in enumerate(self.waypoints)
                if wp.status == WaypointStatus.PENDING
            )
        )
        planned_waypoints = frozenset(
            (
                task_spec
                for robot_plan, progress in zip(
                    self.current_plan, self.current_plan_progress
                )
                for task_spec in robot_plan[progress:]
                if isinstance(task_spec, WaypointID)
            )
        )

        if pending_waypoints != planned_waypoints:
            return "New waypoint added."

        _, plan = self.integrate_plan(robot_states)
        if min((x.remaining_battery for r in plan for x in r), default=1.0) < 0.0:
            return "Not enough battery to finish plan."

        # Plan is valid.
        return None

    def update_plan(self, robot_states: List[RobotState], plan_flaw: str | None):
        new_plan = self.planner_fn(robot_states, self.waypoints)
        self.current_plan = new_plan
        self.current_plan_progress = [0 for _ in new_plan]
        self.current_plan_version += 1
        self.current_plan_reason = plan_flaw
        print(f"New plan: {self.current_plan}")
        for ii in range(len(self.robots)):

            def task_location(task: TaskSpec) -> Location:
                if task == "charge":
                    cl = robot_states[ii].battery_constraint.charger_location
                    if cl is None:
                        raise Exception(
                            "Cannot charge with unknown charger location")
                    else:
                        return cl
                else:
                    waypoint_id = task
                    return self.waypoints[waypoint_id].location

            self.robots[ii].set_plan(
                [
                    (task_id, task_spec, task_location(task_spec))
                    for task_id, task_spec in enumerate(self.current_plan[ii])
                ]
            )

    def set_status(self, s):
        if self.last_status_message == s:
            return
        self.last_status_message = s
        print("Status: ", s)

    def integrate_plan(self, robot_states):
        total_cost = 0.0
        combined_plan = []
        for robot_state, robot_plan, start_idx in zip(
            robot_states, self.current_plan, self.current_plan_progress
        ):
            cost, plan = integrate_robot_plan(
                robot_state,
                self.waypoints,
                robot_plan[start_idx:]
            )
            total_cost += cost
            combined_plan.append(plan)
        return total_cost, combined_plan

    def publish_state(self, robot_states: List[RobotState]):
        total_cost, combined_plan = self.integrate_plan(robot_states)
        planstatus = PlanStatus(
            self.last_status_message or "",
            self.planner_name,
            total_cost,
            combined_plan,
            self.current_plan_version,
            self.current_plan_reason,
        )
        controllerstatus = ControllerStatus(
            self.waypoints, robot_states, planstatus)

        json_output = json_dumps_dataclass(controllerstatus)
        self._mqtt_client.publish("planner/status", json_output, retain=True)


if __name__ == "__main__":
    with open("configuration.json") as f:
        configuration = json.load(f)
    controller = Controller(configuration)

    if configuration.get("autoload-waypoints", False):
        # For some reason the mqtt client is not really listening yet
        time.sleep(0.5)
        load_default_waypoints()

    controller.main_loop()
