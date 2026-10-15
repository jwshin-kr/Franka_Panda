# Franka Bridge

This directory contains the C++ bridge for the Franka Research 3 arm
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

Needs libfranka >= 0.15 (FR3 system >= 5.7.2; 0.17.0 for the DROID cell's
system 5.8.1), yaml-cpp, and the LCM that ships with the `pip install lcm` of
the active conda env. libfranka >= 0.15 needs pinocchio (C++); the `pin` pip
package of the conda env provides it (`cmeel.prefix`).

```bash
conda activate franka
PIN=$CONDA_PREFIX/lib/python3.12/site-packages/cmeel.prefix

# libfranka 0.17.0 -> ~/libfranka/build (bridge_ld_library_path in hardware/default.yaml)
git clone --recursive --branch 0.17.0 https://github.com/frankaemika/libfranka.git ~/libfranka
cmake -S ~/libfranka -B ~/libfranka/build -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_PREFIX_PATH=$PIN
cmake --build ~/libfranka/build -j

# the bridge
cmake -S communication/franka -B communication/franka/build -DCMAKE_BUILD_TYPE=Release \
    -DFranka_DIR=$HOME/libfranka/build -DCMAKE_PREFIX_PATH=$PIN
cmake --build communication/franka/build -j
```

## Run

Usually spawned by `run_hardware.py` (`HardwareFrankaFR3`). Standalone:

```bash
communication/franka/build/franka_bridge configs/franka_fr3/hardware/bridge.yaml
```

## Key Notes

- This bridge is arm-only. It expects a command with at least
  `ctrl_joint_offset + 7` joints (the 8-DoF `[arm(7), gripper(1)]` command) and
  publishes a 7-DoF arm measurement. The Robotiq 2F-85 is driven from Python
  (`robot_platform/hardware/franka_fr3/Robotiq2F85Hardware.py`, Modbus RTU).
- libfranka torque commands are "without gravity": the robot always adds its
  own model gravity. With `enable_gravity_comp: false` the bridge subtracts it
  again, because the controller's `tau_ff` already contains `g(q)`.
- No fresh command (older than `command_timeout_s`) ends the torque control
  loop, so the robot's own controller holds the pose (Idle). The next fresh
  command starts the loop again.
- A reflex, user stop, or network error aborts the control loop; the bridge
  then exits, and `HardwareFrankaFR3` shuts the whole platform down.
- The LCM serialization is implemented locally to match the repo's existing
  `joint_ctrl_t` and `joint_meas_t` wire formats without requiring C++ codegen.
