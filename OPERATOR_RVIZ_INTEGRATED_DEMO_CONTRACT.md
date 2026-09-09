# Operator GUI + Canonical RViz Integrated Demo Contract — MK2G1C

## Purpose and boundary

`operator_visualization_demo.launch.py` is a non-production desktop demo.
It starts one synthetic authority, the existing observation-only RViz helper,
the standalone Operator GUI, and canonical RViz. It does not start PlanNav,
sensors, localization, Nav2, Collision Monitor, the Generic Gate, a chassis
backend, SocketCAN, or a physical robot.

The fixture's synthetic `/vehicle_cmd_safe` is explicitly a *display input*
for Safe Commanded Speed. It is not connected to a lower command path.

## Processes

| Process | Role | Authority |
|---|---|---|
| `integrated_demo_fixture` | sole synthetic route/status/spatial source | demo only |
| `navigation_visualization_helper` | nearest-point and current-goal markers | observation only |
| `operator_gui` | supervisory display; Mission service client | existing service contract only |
| `rviz2` | spatial renderer | display only |

There is exactly one publisher in the demo for each shared retained mission
topic: `/mission/route` and `/mission/state`.

## Scenario

The fixture publishes Mission `demo-route-01` with route nodes `A, B, C, D`,
edges `A->B, B->C, C->D`, and current waypoint index `2`. Consequently both
the GUI Current Goal and RViz goal marker identify pose `C (4.00, 1.00)`.
The final topological Goal Node is `D`; it is not confused with the current
metric goal.

The map, costmap, footprint, robot TF/description, global path, nearest point,
and marker-array trajectories are synthetic visualization data only. The
fixture deliberately publishes `Optimal Trajectory` and `Candidate
Trajectories`; canonical RViz enables only Optimal Trajectory by default.

## Controls and state sequence

The fixture mirrors the accepted Mission Manager service shapes and legal
states:

* START from `RECEIVED` -> `NAVIGATING`.
* PAUSE from `NAVIGATING` -> `PAUSED`.
* RESUME from `PAUSED` -> `NAVIGATING`.
* CANCEL from a cancellable active state -> `CANCELLED`.

The Operator GUI does not mutate its own mission model from a click. It waits
for the fixture's authoritative `/mission/state` update. `initial_state:=FAILED`
is a deterministic visual fault mode, publishing Gate state `DISARMED_ZERO`
and reason `GATE_DISARMED_DEMO`. It has no reset control and grants no motion
authority.

## Desktop policy

The Operator GUI and RViz are separate windows. The GUI answers mission,
navigation, safety, and allowed supervisory actions. RViz answers spatial
questions: robot, map, plan, selected trajectory, goal, nearest point, and
costmap obstacles. Aim Point remains absent by design.

## Run command

```bash
source /opt/ros/humble/setup.bash
source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
ros2 launch robot_visualization operator_visualization_demo.launch.py
```

This command is demo-only and must not be used as a production robot bringup.
