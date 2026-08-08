# G1 mode 13 (5010) + MID360

`G1_29dof_mode_13_5010_mid360.usd` combines Unitree's official mode-13 5010
wrist geometry/inertias/limits, the InstinctLab torso-rooted training collision
contract, and the repository's MID360 non-repetitive scan profile.

The flattened USD has 29 revolute joints, no external USD references, and no
embedded ROS or motion-control Action Graph. Its sensor prim is:

```text
/g1_29dof_mode_13_5010_mid360/torso_link/mid360_link/mid360_native_approx
```

Regenerate with an Isaac Lab environment compatible with Isaac Sim 5.1:

```bash
python scripts/build_g1_5010_mid360_asset.py
python scripts/validate_assets.py
```

Source revisions, file hashes, the 5010 wrist contract, MID360 parameters and
licenses are pinned in `manifest.json` and `source/`.
