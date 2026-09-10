# P5A MK2H1R Classification of Retained MK2H1 Work

No MK2H1 file was removed, reverted, committed, or staged.

## H1_VALID_INDEPENDENT_OF_ROUTE_BLOCKER

- `src/robot_bringup/launch/bringup.launch.py`: opt-in real localization
  validity while preserving the previous default mock profile.
- `src/robot_bringup/launch/mk2h1_live_operator_mock.launch.py`: canonical
  dual-sensor/operator visualization composition with MockTransport.
- `src/robot_bringup/test/test_mk2h1_live_operator_contract.py`: containment,
  real-validity, and no-demo-fixture contracts.
- `src/robot_bringup/CMakeLists.txt`: focused test registration.
- `src/robot_bringup/package.xml`: runtime dependencies for the composed live
  profile.

These changes are useful regardless of which valid canonical route is chosen.

## H1_ROUTE_SPECIFIC

- `src/robot_bringup/launch/bringup.launch.py` now exposes the exact expected
  Mission Manager topology version while retaining `v1` as the predecessor
  default.
- `src/robot_bringup/launch/mk2h1_live_operator_mock.launch.py` pins the H1
  profile to the reconciled canonical topology version.
- `src/robot_bringup/test/test_mk2h1_live_operator_contract.py` verifies that
  binding.
- `P5A_MK2H1_LIVE_OPERATOR_VIS_RUNTIME_CONTRACT.md`
- `P5A_MK2H1_LIVE_DISPLAY_AUTHORITY_MATRIX.csv`
- `P5A_MK2H1_LIVE_TIMING_SUMMARY.csv`

These accurately preserve the blocked run and must remain evidence, but they
are not the future route authority.

## H1_UNCERTAIN

None.

Localization-validity timing parameters remain unchanged. The intermittent
MK2H1 `ODOM_STALE/STABILITY_WAIT` observation is deferred until after a
canonical route is admitted.
