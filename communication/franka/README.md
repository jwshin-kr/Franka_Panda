# Franka Bridge

This directory contains the C++ bridge for the Franka Emika Panda arm
(libfranka), ported from the Flexiv bridge of RoPhiTutorial.

The intended architecture is:

1. Python controllers publish `joint_ctrl_t` over LCM.
2. This bridge subscribes to the control channel in C++.
3. The bridge runs the arm-side loop at 1 kHz; the torque itself is computed
   inside libfranka's 1 kHz control callback.
4. The bridge publishes `joint_meas_t` back over LCM.

The bridge runs in `mock_mode: true` by default so the LCM plumbing, config
loading, joint slicing, and safety fallback can be exercised without a robot.

## Build

Needs libfranka 0.9.x (Panda, system 4.x), yaml-cpp, and the LCM that ships
with the `pip install lcm` of the active conda env.

```bash
conda activate rophi
cmake -S communication/franka -B communication/franka/build \
    -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=$HOME/libfranka/build
cmake --build communication/franka/build -j
```

## Run

Usually spawned by `run_hardware.py` (`HardwareFrankaPanda`). Standalone:

```bash
communication/franka/build/franka_bridge configs/franka_panda/hardware/bridge.yaml
```

## Key Notes

- This bridge is arm-only. It expects a command with at least
  `ctrl_joint_offset + 7` joints (the 9-DoF `[arm(7), hand(2)]` command) and
  publishes a 7-DoF arm measurement. The Franka Hand is driven from Python.
- libfranka torque commands are "without gravity": the robot always adds its
  own model gravity. With `enable_gravity_comp: false` the bridge subtracts it
  again, because the controller's `tau_ff` already contains `g(q)`.
- No fresh command (older than `command_timeout_s`) ends the torque control
  loop, so the robot's own controller holds the pose (Idle). The next fresh
  command starts the loop again.
- A reflex, user stop, or network error aborts the control loop; the bridge
  then exits, and `HardwareFrankaPanda` shuts the whole platform down.
- The LCM serialization is implemented locally to match the repo's existing
  `joint_ctrl_t` and `joint_meas_t` wire formats without requiring C++ codegen.
