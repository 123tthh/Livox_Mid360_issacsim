# G1 rev 1.0 (4010) + MID360

`G1_29dof_mid360.usd` is the self-contained 29-DoF G1 4010 baseline
(`mode_machine=5`). It has 29 revolute joints and no external USD references.
Its Isaac Sim 5.1 `OmniLidar` uses a fixed-size runtime state backed by the
embedded 40-state, four-second Livox reference trajectory at 200,000 rays/s
and 10 Hz. The ROS publisher drives the state without modifying the root USD.
The sensor prim is:

```text
/g1_29dof_rev_1_0/torso_link/mid360_link/mid360_native_approx
```

The asset intentionally contains no saved ROS or motion-control Action Graph.
Use `../../scripts/publish_mid360_ros2_isaacsim51.py` from Isaac Sim's Script
Editor. Exact USD and sensor-contract hashes are in `manifest.json`.
