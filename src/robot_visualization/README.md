# robot_visualization

Observation-only RViz navigation visualization for the accepted Phase-5A
architecture.

The helper subscribes to `/plan`, TF, `/mission/route`, and `/mission/state`
and publishes only RViz markers:

* `/visualization/nearest_path_point`
* `/visualization/current_navigation_goal`

It has no velocity publisher, action client, service client, CAN access, or
mission-control authority. `visualization_fixture` is an offline-only source
of synthetic map/path/mission/TF data and also publishes no motion command.
