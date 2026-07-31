# Phase 2 Map Scenario Selection

Date: 2026-08-01  
Workspace: `/home/dog/fastlio_rtabmap_ros2_ws`

## Source map

| Item | Value |
|---|---|
| Source YAML | `scripts/offline_nav_maps/clean_map.yaml` |
| Source PGM | `scripts/offline_nav_maps/clean_map.pgm` |
| Packaged YAML | `src/parking_robot_bringup/maps/phase2_clean_map.yaml` |
| Packaged PGM | `src/parking_robot_bringup/maps/phase2_clean_map.pgm` |
| PGM SHA256 | `69428c94a8032fd54939492afe848156ef1687faf7a8023609a3e26d9604beb0` |
| Resolution | 0.05 m/cell |
| Origin | [-39.1000, -85.1500, 0.0000] |
| Dimensions | 1744 x 2683 cells |
| Occupied threshold | 0.65 |
| Free threshold | 0.196 |

The PGM copy is byte-identical. The copied YAML image reference was changed to `phase2_clean_map.pgm` so the installed YAML resolves inside the package.

## Static analysis method

The PGM was parsed directly without ROS runtime. Cell probability followed the ROS/map_server trinary convention with `negate: 0`:

`occupancy_probability = (255 - pixel_value) / 255.0`

Classification:

- free: probability `< 0.196`
- occupied: probability `> 0.65`
- unknown: otherwise

An 8-connected-component pass found:

| Class | Count |
|---|---:|
| free cells | 584287 |
| occupied cells | 22595 |
| unknown cells | 4072270 |
| free-space components | 1 |
| largest free component area | 584287 |

Distance transform over the connected free component was used to choose conservative free-space poses. Pixel-to-map conversion used:

- `map_x = origin_x + (pixel_x + 0.5) * resolution`
- `map_y = origin_y + (height - pixel_y - 0.5) * resolution`

## Selected scenarios

| Scenario | Pixel x | Pixel y | Map x | Map y | Yaw | Classification | Component | Clearance evidence |
|---|---:|---:|---:|---:|---:|---|---:|---:|
| start | 890 | 2054 | 5.425 | -53.725 | 0.0 | free | 1 | 7.393 m from non-free |
| goal_a | 1254 | 2602 | 23.625 | -81.125 | 0.0 | free | 1 | 5.430 m from non-free |
| goal_b | 788 | 2083 | 0.325 | -55.175 | 0.0 | free | 1 | 5.437 m from non-free |
| goal_c | 876 | 2147 | 4.725 | -58.375 | 0.0 | free | 1 | 5.423 m from non-free |
| planner_failure_goal | 621 | 1154 | -8.025 | -8.725 | 0.0 | occupied | 0 | occupied probability 1.0; 0.160 m inside occupied region |

The success start/goals are all in the same connected free-space component and do not depend on unknown cells. The failure goal is a confirmed occupied cell and is not part of the free-space component.

## Runtime status

`NOT_STARTED`

These are static scenario selections only. No Nav2 runtime, goal sending, robot motion, or success/failure runtime claim has been made.

