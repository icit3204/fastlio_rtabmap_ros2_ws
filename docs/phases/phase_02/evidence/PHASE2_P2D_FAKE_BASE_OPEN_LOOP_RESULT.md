# Phase 2 P2-D Fake-Base Open-Loop Result

Decision: `OPEN_LOOP_PASS`

Runtime root: `/home/dog/phase2_runtime/p2d_fake_base_open_loop_20260801_044920/`

Only `phase2_fake_base` and the temporary probe node were run. No Nav2 planner, controller, BT navigator, map server, lifecycle manager, hardware, CAN, UDP, or sensor process was started.

## Source behavior checked

- `Twist.linear.x` is treated as body-forward velocity.
- `Twist.angular.z` is treated as yaw rate; positive values increase yaw counter-clockwise.
- Integration uses the exact differential-drive arc equations for nonzero angular rate.
- `/Odometry` and `odom -> base_footprint` TF are published from the same pose and quaternion.
- Command timeout behavior zeroes the integrated command after stale/missing input; the diagnostic published an explicit zero command after each 2 s command window before confirming the stationary state.

## Results

| Case | Expected | Measured at command end | Error | Pass |
|---|---:|---:|---:|---:|
| A_straight_positive_x | x=0.2, y=0.0, yaw=0.0 | x=0.200003108, y=0.000000000, yaw=0.000000000 | dx=3.108e-06, dy=0.000e+00, dyaw=0.000e+00 | True |
| B_positive_rotation | x=0.0, y=0.0, yaw=0.4 | x=0.000000000, y=0.000000000, yaw=0.400010170 | dx=0.000e+00, dy=0.000e+00, dyaw=1.017e-05 | True |
| C_negative_rotation | x=0.0, y=0.0, yaw=-0.4 | x=0.000000000, y=0.000000000, yaw=-0.399994853 | dx=0.000e+00, dy=0.000e+00, dyaw=5.147e-06 | True |
| D_positive_curve | x=0.1947, y=0.0395, yaw=0.4 | x=0.194728302, y=0.039477592, yaw=0.400041542 | dx=2.830e-05, dy=-2.241e-05, dyaw=4.154e-05 | True |

All cases satisfied the requested tolerances: translation ±0.03 m, yaw ±0.04 rad, and TF/odometry agreement <= 1e-6.

Conclusion: fake-base kinematic sign convention and odometry/TF interface are not the primary root cause of the P2-D closed-loop failure.
