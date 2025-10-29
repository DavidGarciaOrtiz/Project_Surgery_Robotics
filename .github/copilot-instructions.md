## Quick context

This repository contains a multi-part surgery-robotics project with several PlatformIO microcontroller projects and RoboDK-based simulation scripts. Key folders:

- `src/` — many device projects (Endowrist, Gripper, Servos, etc.), each a PlatformIO project with its own `platformio.ini` and `src/` code.
- `src/roboDK/` — Python glue for simulation, including `Init_SurgeryRobotics_simulation_v1.py` and RoboDK station `SurgeryRobotics.rdk`.
- top-level `Gripper_IMU/` — another PlatformIO project used for IMU-based testing.

Make decisions using these facts: RoboDK (Python API) drives the simulated robot; IMU/microcontroller devices communicate via UDP JSON messages to the simulator on port 12345.

## High-level architecture (what to know immediately)

- Microcontrollers (PlatformIO projects under `src/<component>/`) stream IMU/button data over UDP to the host. Example JSON payloads seen in `Init_SurgeryRobotics_simulation_v1.py`:

```json
{ "device": "G5_Endo", "roll": <num>, "pitch": <num>, "yaw": <num>, "s3": <0|1>, "s4": <0|1> }
{ "device": "G5_Gri",  "roll": <num>, "pitch": <num>, "yaw": <num>, "s1": <0|1>, "s2": <0|1> }
```

- The simulation script `Init_SurgeryRobotics_simulation_v1.py`:
  - Binds UDP on 0.0.0.0:12345 and parses JSON.
  - Maps Endowrist (G5_Endo) orientation to the robot tool (UR5e) using RoboDK API (`robodk.robolink`, `robodk.robomath`).
  - Updates a Gripper object pose locally in the RoboDK station and re-parents `Needle` depending on gripper button states.

## Developer workflows & commands (explicit)

- Build / upload embedded projects (PlatformIO):
  - From project folder (e.g. `src/Endowrist/`):
    - Build: `platformio run`
    - Build & upload: `platformio run -t upload`
    - Or use the VS Code PlatformIO extension (recommended for device flashing/debugging).

- Run the RoboDK simulation script (Windows PowerShell examples):
  - Pre-reqs: RoboDK app installed and the Python `robodk` module available (RoboDK includes `robodk` or use RoboDK Python package).
  - Open RoboDK and load `src/roboDK/SurgeryRobotics.rdk` (station used by scripts).
  - In PowerShell (from `src/roboDK/`):
    - `python Init_SurgeryRobotics_simulation_v1.py`
  - Notes:
    - Script binds UDP port 12345 (0.0.0.0). Ensure firewall allows UDP on that port.
    - The script expects RoboDK to be reachable (it uses `Robolink()` to connect).

- Quick debug loop used by scripts:
  - The simulation runs two threads: UDP reader and a RoboDK mover thread. Look for `data_lock` usage and `sock.recvfrom` when investigating lost messages or threading races.

## Project-specific conventions & patterns

- Per-component PlatformIO subprojects: each device sits in `src/<Name>/platformio.ini`. When adding a new device, copy an existing folder (e.g. `Gripper_IMU_v1`) and update the `platformio.ini` envs.
- Naming convention: `Endowrist`, `Gripper`, `* _v1/_v2` indicate iterations — prefer creating a new versioned folder rather than changing stable histories.
- RoboDK items referenced by name from scripts: `UR5e`, `UR5e Base`, `Endowrist`, `Gripper`, `Needle`, and `Init`. If renaming in station, update scripts accordingly.

## Integration points and external dependencies

- RoboDK Python API (robodk) — used heavily in `src/roboDK/`.
- UDP JSON messages (port 12345) — interface between firmware and simulation. Devices identify themselves with `device` field.
- PlatformIO / embedded toolchains for building firmware.

## Concrete examples & patterns (from code)

- Transforming IMU angles to RoboDK poses (see `endowrist2base_orientation` and transformation chain in `move_robot`):
  - Endowrist yaw is adjusted by a UI-controlled `ZERO_YAW_TOOL` slider before applying `rotz`.
  - Z-axis relative moves triggered by buttons `s3`/`s4` are applied with `transl(0,0,±5)` and verified with `robot.MoveL_Test(...)`.

- Gripper behavior:
  - `s1` (open) -> `needle.setParentStatic(base)` releases the needle.
  - `s1` (closed) -> `needle.setParent(gripper)` attaches the needle and zeroes its pose.

## What an AI agent should do first (practical checklist)

1. Open `src/roboDK/Init_SurgeryRobotics_simulation_v1.py` to see runtime behavior and UDP schema (it's the fastest way to learn control flow).
2. Open `src/roboDK/SurgeryRobotics.rdk` in RoboDK to inspect named Items used by scripts (`UR5e`, `Endowrist`, `Gripper`, `Needle`).
3. When working on device firmware, run PlatformIO builds from the device folder to confirm envs in `platformio.ini`.
4. To test integration locally without hardware, run the Python script and stream synthetic UDP JSON messages to 127.0.0.1:12345 matching the shapes above.

## Edge cases & pitfalls to watch for (observed in code)

- Threading and UDP socket lifetime: the UDP read loop closes the socket on errors and threads are daemonized; confirm graceful shutdown when closing the Tk GUI.
- RoboDK MoveL reachability is checked with `MoveL_Test(...)` — if unreachable, scripts only display a message and skip the move.
- JSON parsing: code quietly ignores decoding errors — expect malformed packets in noisy networks.

## Where to add tests & quick wins

- Add small Python unit tests for the angle-to-pose conversion functions (e.g., `endowrist2base_orientation`) under a tests folder in `src/roboDK/`.
- Add a lightweight UDP test harness script (send sample JSON payloads) to `src/roboDK/test/` so CI or devs can replay scenarios.

---
If anything here is out-of-date or you want more detail (for example, the exact `platformio.ini` env names to reference in CI, or the RoboDK station layout), tell me which area to expand and I will iterate.
