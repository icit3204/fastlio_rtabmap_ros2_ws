# OpenCV 4.5.4d Feature Availability Audit

The stable candidate uses OpenCV 4.5.4d with RTAB-Map
`WITH_TORCH=OFF` and `WITH_PYTHON=OFF`. The recovered Ubuntu OpenCV package
set does not provide xfeatures2d/nonfree.

Three tracked authoritative robot bringup launch files actively select
features unavailable in this stable prefix:

- `src/robot_bringup/launch/fastlio_mapping_infra_2d.launch.py`:
  `Kp/DetectorStrategy=11`, `Vis/FeatureType=11`,
  `SuperPoint/Cuda=true`, SuperPoint model path,
  `PyMatcher/Cuda=true`, PyMatcher script path, and model `outdoor`.
- `src/robot_bringup/launch/bringup_2d.launch.py`:
  `Kp/DetectorStrategy=11`, `Vis/FeatureType=11`,
  SuperPoint model path, `PyMatcher/Cuda=true`, and PyMatcher script path.
- `src/robot_bringup/launch/bringup_2d_infra.launch.py`:
  `Kp/DetectorStrategy=11`, `Vis/FeatureType=11`,
  SuperPoint model path, `SuperPointRpautrat/Cuda=true`,
  `PyMatcher/Cuda=true`, and PyMatcher script path.

These active defaults must be corrected in a later configuration phase
before those camera launch modes are used. They were not edited here. This
does not block the LiDAR-centered native build.

The stereo outdoor demo contains only commented xfeatures2d-dependent
strategy values. The multisession demo actively uses SIFT strategy 1, which
is available. The find-object demo configuration lists several selectable
features, including unavailable SURF/SuperPoint choices; these are optional
demo settings rather than robot runtime defaults. FAST-LIO uses of “surf”
refer to LiDAR surface points and are unrelated to OpenCV SURF.

Torch SuperPoint and Python feature strategies are deferred, not abandoned.
Camera hardware, camera topics, ordinary OpenCV processing, classical
features, recording/replay, RViz image viewing, and multi-sensor bag data
are not removed.
