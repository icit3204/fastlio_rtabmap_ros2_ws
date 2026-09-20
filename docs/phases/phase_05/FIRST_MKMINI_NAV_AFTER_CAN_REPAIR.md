# First MK-mini navigation after CAN repair

Qualified software checkpoint: `b085084164b816e9b6ff161d8bd8e4fd14a1abcc`  
Freeze tag: `phase5a_r3l_r23_r6_live_integration_pass`

## Precondition

Repair CAN and prove the receive path healthy. Keep the current MK-mini commissioning speed ceiling. Do not enable chassis-feedback interlocking: vendor/chassis feedback qualification remains explicitly deferred for this temporary demo platform.

After R23-R7 qualification, the physical R3H launch enforces the selected motion-aware output through the existing Generic Gate. Start it explicitly with:

```bash
source /opt/ros/humble/setup.bash
source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
ros2 launch parking_robot_bringup r3h_physical_navigation.launch.py collision_monitor_mode:=motion_aware_experimental
```

## Practical first-motion sequence

1. Start one clean R3H stack only. Verify the singleton guard, localization, TF, MID-360, and T-mini.
2. Verify the R22 local/global costmaps are clear at the stationary start.
3. Explicitly select `motion_aware_experimental`. Require the startup indicator `ACTIVE_COLLISION_MODE=motion_aware_experimental`, then verify exactly one collision authority, one Generic Gate output authority, and one physical backend; no mock or competing command publisher may remain.
4. Verify stationary command and measured motion are zero before enabling motion.
5. Compute the planner path from the current pose to WP01 and require PASS.
6. Navigate first to WP01 `(2.878548, 0.102724, 8.733 deg)`. Continue to WP02 `(4.472000, 0.303000, 10.000 deg)` only if WP01 tracking, localization, perception, Gate state, and CAN health remain sound.
7. Record the actual FAST-LIO trajectory against the planned path.
8. With the robot stopped, test an obstacle outside the swept route, then one inside it. Confirm Nav2 reroutes or returns NO-PATH as appropriate, while motion-aware SLOW/STOP remains independent and fail-closed.
9. End with command zero, confirm physical speed zero, stop application CAN TX, and shut down the stack cleanly.

## One-command fixed rollback

The currently supported physical authority remains the qualified fixed mode:

```bash
source /opt/ros/humble/setup.bash
source /home/dog/fastlio_rtabmap_ros2_ws/install/setup.bash
ros2 launch parking_robot_bringup r3h_physical_navigation.launch.py collision_monitor_mode:=fixed_qualified
```

Do not merge the development branch to `main` until the first stationary physical-backend preflight is separately qualified after CAN repair.
