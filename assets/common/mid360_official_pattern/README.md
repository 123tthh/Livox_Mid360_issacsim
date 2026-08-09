# MID-360 reference trajectory

`mid360.csv` is the unmodified `scan_mode/mid360.csv` file from
[`Livox-SDK/livox_laser_simulation`](https://github.com/Livox-SDK/livox_laser_simulation),
commit `1cce1073633a062b92e30243a4c2920e45551bb5`. Its SHA-256 is
`aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a`.
The upstream file and simulator are MIT-licensed; the retained license is
`LICENSE.livox_laser_simulation`.

The 800,000 ordered directions represent four seconds at the MID-360's
official 200,000 points/s rate. The upstream first-column heading says
`Time/s`, but its values are sequential sample indices and the upstream Gazebo
plugin does not use them as seconds. This repository therefore reconstructs
timing at 5 microseconds per point from the device specification.

Isaac Sim elevation is derived as `90 degrees - Zenith/deg`. The trace reaches
approximately -7.2123 to 52.164 degrees around the nominal -7 to 52 degree
vertical FOV. Forty consecutive 20,000-ray emitter states preserve the native
10 Hz output cadence, ordered per-ray fire times, and the
non-repeating coverage growth within the four-second reference window.
