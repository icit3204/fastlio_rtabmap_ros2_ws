# Current-room map-to-navigation operator runbook

Status: superseded for current MK-mini work by the R3H-qualified mapping path on 2026-09-15. Historical commands below remain evidence only; do not reuse their legacy static transform or `phase5_calibrated_localization.launch.py` for current-room production work.

## R3H-qualified current-room path

1. Before creating a session, confirm MID-360 Ethernet RX is increasing on `enP8p1s0` (`192.168.168.50/24`), the MID-360 at `192.168.168.20` responds, `/livox/lidar` advances at approximately 10 Hz, `/livox/imu` at approximately 200 Hz, and FAST-LIO `/Odometry` advances at approximately 10 Hz.
2. Use the committed `mkmini_calibrated_localization.launch.py` current calibrated body-to-`base_footprint` authority. Preserve raw FAST-LIO `odom -> body`; RTAB must use `frame_id=base_footprint`, native TF odometry `odom_frame_id=odom_chassis`, and isolated RTAB IMU `/unused_imu`.
3. Confirm RTAB mapping uses `Grid/3D=true`, `Grid/NormalsSegmentation=false`, `Grid/MinGroundHeight=-0.20`, `Grid/MaxGroundHeight=0.15`, `Grid/MaxObstacleHeight=2.0`, and `Grid/RayTracing=true`.
4. On this temporary MK-mini platform only, enable the RTAB-only self-filtered cloud branch. It removes the measured cooler/collar self-volume in `base_footprint`: X `0.540..0.670 m`, Y `-0.140..0.150 m`, Z `0.530..0.580 m`. Raw `/cloud_registered_body`, FAST-LIO, and safety perception topics remain unchanged. Requalify or disable this crop on any final robot.
5. Before manual mapping movement, verify filtered RTAB cloud timestamps/frame are healthy and it contains zero points in that self-volume while raw external returns outside it remain present.
6. After mapping, require planar trajectory, free-cell audit, local-grid self-return audit, and visual review before accepting the DB. Do not promote a map with self-body breadcrumbs or unexplained occupied speckles in clear observed floor.
7. Localize only against a runtime copy. Seed the new map with native `/initialpose`; visually confirm map-to-`base_footprint`, then manually validate a 0.5–1.0 m move.
8. Screen live start and every waypoint using the full production footprint: zero unknown, lethal, and inscribed footprint cells. Use the unchanged Smac Hybrid DUBIN planner (`minimum_turning_radius=1.75 m`, `allow_unknown=false`) as the topology edge oracle. Validate live-start-to-first-waypoint and every persisted directed edge before any later mission milestone.

1. Keep physical CAN disconnected. Connect the MID-360 and its IMU. The T-mini is not required for map acquisition or this localization check.
2. Choose a unique session name in `YYYYMMDD_HHMMSS_label` form and create `map/current_room_sessions/<SESSION_ID>/`. Never reuse an existing directory.
3. Record the historical map hash before starting:

   ```bash
   cd /home/dog/fastlio_rtabmap_ros2_ws
   sha256sum map/rtabmap_2d.db
   ```

4. In terminal A, source ROS and the workspace, then publish the accepted calibrated body transform:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
   ros2 run tf2_ros static_transform_publisher --x 0.452466 --y -0.183290 --z -1.396285 --roll 0.0 --pitch -0.49741883681838395 --yaw 0.0 --frame-id body --child-frame-id base_footprint
   ```

5. In terminal B, run the mapping command, replacing `<SESSION_ID>` once. This exact launch form was validated in the first run:

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
   ros2 launch robot_bringup fastlio_mapping.launch.py database_path:=/home/dog/fastlio_rtabmap_ros2_ws/map/current_room_sessions/<SESSION_ID>/rtabmap_2d.db delete_db_on_start:=true imu_topic:=/livox/imu rviz:=false rtabmap_viz:=false
   ```

6. Mapping is ready only after the MID-360 streams, FAST-LIO reports IMU initialization and map-kdtree initialization, `/Odometry` advances, and RTAB prints continuing `rtabmap (N)` updates.
7. Push the robot manually and slowly on the floor. Capture walls, corners, openings, and usable corridors. Do not lift, shake, tilt, or command powered motion.
8. Keep the same mapping session active for both the outward and return passes. The return overlap supplies additional spatial constraints and helps expose inconsistent closure candidates.
9. Back near the start, stop terminal B with Ctrl-C and wait for RTAB to report that database saving is done. Then stop terminal A.
10. Verify the new database and historical database independently:

    ```bash
    sha256sum map/current_room_sessions/<SESSION_ID>/rtabmap_2d.db map/rtabmap_2d.db
    ```

11. Copy the saved database to a unique temporary directory before localization. The successful localization command was:

    ```bash
    cp map/current_room_sessions/<SESSION_ID>/rtabmap_2d.db /tmp/<UNIQUE_RUNTIME_DIR>/rtabmap_2d.db
    source /opt/ros/humble/setup.bash
    source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
    ros2 launch robot_bringup phase5_calibrated_localization.launch.py database_path:=/tmp/<UNIQUE_RUNTIME_DIR>/rtabmap_2d.db
    ```

12. Confirm `/map`, `/Odometry`, and `map -> base_footprint`. Push the robot a short distance manually and verify continuous, bounded movement in the saved map. Stop with Ctrl-C after validation.
13. Verify the saved database hash before PlanNav import:

    ```bash
    cd /home/dog/fastlio_rtabmap_ros2_ws
    sha256sum map/current_room_sessions/<SESSION_ID>/rtabmap_2d.db
    ```

14. Start PlanNav with the command qualified during the optimized-import run, then click **`导入优化 map DB`** and select the exact saved `rtabmap_2d.db`:

    ```bash
    cd /home/dog/fastlio_rtabmap_ros2_ws
    DISPLAY=:0 PYTHONPATH=plan_nav python3 plan_nav/main.py
    ```

    Do not click the historical `导入 .db 文件` button for new map-frame authoring; that button intentionally retains raw `Node.pose` semantics.
15. Confirm the PlanNav log says `OPTIMIZED_RTAB_GRAPH / frame=map` and displays the expected DB SHA prefix. The optimized workflow creates `map/current_room_sessions/<SESSION_ID>/plannav/plannav_workspace_manifest.json`; verify that manifest records the exact DB path/hash, session ID, `frame=map`, and `OPTIMIZED_RTAB_GRAPH` semantic.
16. Place a small number of nodes only on recognizable, mapped free-space locations. Review their map-frame x/y/yaw and directed edges before saving. Saving must generate persisted edge IDs and an exact topology version.
17. Build and validate a typed RouteMission offline. A planner-only preflight from live TF to the exact first persisted node must pass before any later RouteMission execution or Mission START.
18. Before a later navigation run, verify the exact DB hash, topology version, live localization, perception validity, planner reachability, MockTransport-only containment, and zero physical CAN authority.
19. Shut down PlanNav, RViz, RTAB, FAST-LIO, and the LiDAR driver; verify no ROS nodes or SocketCAN writer remains and CAN TX has not changed.

## Validated first-run identity

- Session: `20260911_194712_current_room`
- Saved DB: `/home/dog/fastlio_rtabmap_ros2_ws/map/current_room_sessions/20260911_194712_current_room/rtabmap_2d.db`
- SHA-256: `550227ccf65470946d78317f28f200f3dcc904dfa053291f69f5f06b67fdd127`
- Historical DB SHA-256: `73788305089e9ceb302ae8a68c3164ca35e2e1cd985458b755eeedee5efadaed`
- PlanNav workspace: `/home/dog/fastlio_rtabmap_ros2_ws/map/current_room_sessions/20260911_194712_current_room/plannav`
- Coordinate semantic: `OPTIMIZED_RTAB_GRAPH`, frame `map`
- Approved topology version: `sha256:948217a0129dbf785b4364cdff94fe5424563719495a6aeb1bfc2780814d1a2e`
- Validated offline route: `1 -> 2 -> 3` using `edge-000001`, `edge-000002`
