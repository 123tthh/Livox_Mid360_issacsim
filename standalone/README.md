# Livox MID-360 standalone asset for Isaac Sim 5.1

Import or reference `MID360_nonrepetitive.usd`. It contains:

- the MID-360 CAD assembly from `mid-360-asm.stp` (22 source meshes, 77,565 triangles);
- an Isaac Sim 5.1 `OmniLidar` configured for 200,000 points/s and 10 Hz;
- all 800,000 directions from the official Livox four-second non-repetitive scan table.

Keep `mid360_cad.usd` beside the wrapper because it is referenced relatively.
The wrapper stage uses metres and Z-up; the CAD millimetre coordinates are
scaled by 0.001. Its default prim is `/MID360`, and the sensor is at
`/MID360/torso_link/mid360_link/mid360_native_approx`.

To animate all 40 scan states, open the USD in Isaac Sim 5.1, run the entire
`drive_mid360_nonrepetitive_isaacsim51.py` file from Window > Script Editor,
then press Play. Call `cleanup_mid360_pattern_driver()` to stop it.

The scan table is redistributed under the Livox SDK2 BSD-3-Clause license; see
the source repository's `assets/mid360_pattern/LICENSE.livox_laser_simulation`.
The CAD geometry is derived from the repository owner's supplied STEP assembly
and is included here as the standalone Isaac Sim import asset requested for
this project.
