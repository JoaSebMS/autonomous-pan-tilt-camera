# Hardware

## 3D-printed gimbal

The pan-tilt gimbal used in this project is **not my design**. I printed the model
published on Thingiverse:

> **[SG90 Servo 2-axis Gimbal](https://www.thingiverse.com/thing:2892903)** (thing #2892903)
> by **aanarcissus** · License: **Creative Commons – Attribution – Share Alike (CC BY-SA)**
> ([license text](stl/LICENSE-gimbal.txt))

The three parts in [`stl/`](stl/) are the files I printed. They are **unmodified copies** of the
originals, included here for convenience under the same CC BY-SA license:

| File | Part |
|------|------|
| `stl/base.stl` | Base |
| `stl/HolderL.stl` | Left holder |
| `stl/HolderR.stl` | Right holder |

All credit for the mechanical design goes to aanarcissus. The CC BY-SA license applies to these
STL files only; the project's source code is MIT-licensed (see the root [`LICENSE`](../LICENSE)).

## Bill of materials

| Component | Role |
|-----------|------|
| Raspberry Pi 4 Model B | Runs the Python software, camera capture and YOLOv8 inference |
| Raspberry Pi Camera Module Rev 1.3 (OV5647) | Video input over the CSI ribbon (used due to limited resources; Camera Module 2 is also supported) |
| PCA9685 16-channel PWM / servo driver | Generates the servo PWM over I²C |
| 2 × SG90 micro servos | Pan and tilt (the gimbal is designed for SG90 servos) |
| Breadboard power module + 9 V battery | Separate servo supply (5 V rail), common ground with the Pi |
| Jumper wires, cable ties | Wiring and mounting the camera to the gimbal |

## Assembly notes

- Camera is cable-tied to the tilt platform (see `docs/images/hardware/gimbal-mount-closeup.jpg`).
- Run [`tools/servo_calibration.py`](../tools/servo_calibration.py) **before** the first
  autonomous run and set `PAN_ANGLE_LIMITS` / `TILT_ANGLE_LIMITS` in `src/config.py`,
  so the ribbon cable and servo wires are never over-bent and the printed parts never bind.
