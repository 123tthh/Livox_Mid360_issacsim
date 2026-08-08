# Third-party notices

This repository contains derived robot assets in addition to MIT-licensed
repository code. The root `LICENSE` does not replace the licenses below.

- Unitree G1 description and meshes: `unitreerobotics/unitree_ros`, commit
  `f3772ce54c56ef2d34c6aee8100bc768896c7d19`, BSD-3-Clause. The full license is
  retained at `assets/g1_29dof_mode_13_5010/source/unitree_ros/LICENSE`.
- InstinctLab torso-rooted training collision model: `project-instinct/InstinctLab`,
  commit `ba28d3d2655b15a19b729476a630937a19610a3b`, CC BY-NC 4.0. The full license
  is retained at `assets/g1_29dof_mode_13_5010/source/instinctlab/LICENSE`.
- Livox MID-360 reference scan trajectory: `Livox-SDK/livox_laser_simulation`,
  commit `1cce1073633a062b92e30243a4c2920e45551bb5`, MIT. The unmodified
  `scan_mode/mid360.csv` and its license are retained under
  `assets/mid360_pattern/`.
- NVIDIA Isaac Sim schemas and runtime are external dependencies and are not
  redistributed here.
- Livox and Unitree names are used only to identify compatible hardware/model
  families; this repository does not claim endorsement by either vendor.

The 5010 USD is an adapted, flattened asset. Its source commits and hashes are
recorded in `assets/g1_29dof_mode_13_5010/manifest.json`. The 4010 baseline
origin and capture status are recorded in its manifest.
