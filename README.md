# isar-inspectionroute

This repository contains experimental code for real-time planning of a robot
mission to perform task at different locations.  The objective of the planning
is to plan and execute the tasks, including traveling to the different
locations where the tasks are performed.  The robot also needs to always get
back to a charging station before fuel/battery runs out. When real-time
information is received, the plan is updated to take the new information into account.

## Architecture

The front-end interface is the GUI where the operator can enter tasks with
corresponding locations.

The back-end interface is the [ISAR](https://github.com/equinor/isar) robot
controller, accessed through MQTT and HTTP for commanding the robot to execute
the tasks and to receive real-time data on the mission status, and also current
location and battery level.

```
  Front-end: (GUI)                               Back-end: (ISAR)
  * Locations, tasks    ┌──────────────────────┐ ◄───────────
 ──────────────────────►│ isar-inspectionroute │   Status, location, battery
                        └──────────────────────┘ ────────────►
                           Task sequencing         Mission sequence
```

## Implementations

There are two implementations:

 * **`greedy`** is a Python implementation of a greedy choice of waypoint sequence. It is meant to be used mainly for ISAR interface testing, and does not have a GUI.
 * **`interactive`** is a Rust GUI application. It performs a heuristic optimization of the waypoint sequence that is updated on demand.


