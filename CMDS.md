# Run commands

Every mode uses 3 terminals, started in order 1 → 2 → 3. Set up each terminal first:

```bash
conda activate rophi
cd ~/jwjh/Franka
```

- Viser: http://localhost:8080
- Real robot: start terminal 2 only after terminal 1 prints `[Hardware] Ready`. Keep the user stop in hand.
- Quit: press `q` in the controller terminal (2), then Ctrl-C in terminals 3 and 1.
- Always use the `*_hw.yaml` visualizer config with the real robot. A sim visualizer config shows nothing.

---

## Simulation (MuJoCo)

### Gravity compensation (01–03)
```bash
python run_env.py --config configs/franka_fr3/env/default.yaml                          # 1
python run_controller.py --config configs/franka_fr3/controllers/grav_comp.yaml        # 2
python run_visualizer.py --config configs/visualizer/franka_fr3/default.yaml           # 3
```

### Collision-aware gravity compensation, v2 (05)
```bash
python run_env.py --config configs/franka_fr3/env/collision.yaml                                  # 1
python run_controller_v2.py --config configs/franka_fr3/controllers/col_aware_grav_comp_v2.yaml  # 2
python run_visualizer_v2.py --config configs/visualizer/franka_fr3/collision_v2.yaml             # 3
```

### IK QP, v2 (06)
Keys (terminal 2): `w/s` ±x, `a/d` ±y, `r/f` ±z, `[`/`]` step size, `h` home, `o`/`c` gripper open/close.
```bash
python run_env.py --config configs/franka_fr3/env/collision.yaml                                 # 1
python run_controller_v2.py --config configs/franka_fr3/controllers/ik_qp.yaml --name ik_qp_v2  # 2
python run_visualizer.py --config configs/visualizer/franka_fr3/ik_qp.yaml                      # 3
```

### Impedance, v2 (08)
Keys (terminal 2): `j` joint impedance, `t` task (SE(3)) impedance. Tune gains with the Viser sliders.
```bash
python run_env.py --config configs/franka_fr3/env/default.yaml                                          # 1
python run_controller_v2.py --config configs/franka_fr3/controllers/impedance.yaml --name impedance_v2  # 2
python run_visualizer.py --config configs/visualizer/franka_fr3/impedance.yaml                         # 3
```

### Differentiable base–camera calibration (14)
Sim only, no visualizer; terminal 3 is the calibration. Drag the arm in the MuJoCo viewer until it is visible in the
**Calib RGB** window, press `SPACE` there to capture (`num_captures`, ESC finishes early with ≥ 2), left-click the robot in
the SAM2 window and press `ENTER` (`R` resets clicks). When the optimization ends, press any key in the mosaic window;
the terminal prints `T_cam2base`, `T_world2cam` and the error vs the MuJoCo ground truth.
```bash
python run_env.py --config configs/calibration_tutorial/env/franka_arm_hand_cam.yaml                     # 1
python run_controller.py --config configs/calibration_tutorial/controllers/grav_comp.yaml               # 2
python run_camera_calibration.py --config configs/calibration_tutorial/calib_diff_render.yaml           # 3
```
`run_camera_calibration_v2.py` (same arguments) runs the same calibration and also saves the result to
`configs/calibration_tutorial/results/calib_camera.yaml` (`--output` to change), which the point tracking below uses.

### Click-and-track pointing (perception + IK QP v2)
First run the calibration above with `run_camera_calibration_v2.py` (it writes the `T_cam2base` that terminal 3 uses).
Then click a point on the target cube in the **Click on Image** window; the gripper (tcp) follows that point,
10 cm above it. Move the cube in the MuJoCo viewer: double-click it, then Ctrl + right drag (Ctrl + left drag rotates).
Keys (terminal 2): `t` follow on/off, `h` home, `o`/`c` gripper; with following off, the IK QP keys move the target.
Click again at any time to track a new point. Terminal 4 (optional) shows the target and tip in Viser.
```bash
python run_env.py --config configs/point_tracking/env/franka_arm_hand_col_cam.yaml                        # 1
python run_controller_v2.py --config configs/point_tracking/controllers/ik_qp_point_follow.yaml          # 2
python run_perception.py --config configs/point_tracking/perception/tapnet_click_and_track.yaml          # 3
python run_visualizer.py --config configs/visualizer/franka_fr3/ik_qp.yaml                               # 4 (optional)
```

---

## Real robot (FR3 + Robotiq 2F-85)

Terminal 1 is the same for every mode. It moves the arm to the start pose `[0, 0, 0, -90, 0, 90, 0]` deg first.

### Gravity compensation (01–03): try this one first
```bash
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml                 # 1
python run_controller_v2.py --config configs/franka_fr3/controllers/grav_comp_hw.yaml   # 2
python run_visualizer.py --config configs/visualizer/franka_fr3/default_hw.yaml         # 3
```

### Collision-aware gravity compensation, v2 (05)
Uses the virtual walls of the sim cell (floor, wall_back, wall_left), not the real DROID cell.
```bash
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml                              # 1
python run_controller_v2.py --config configs/franka_fr3/controllers/col_aware_grav_comp_v2_hw.yaml   # 2
python run_visualizer_v2.py --config configs/visualizer/franka_fr3/collision_v2_hw.yaml              # 3
```

### IK QP, v2 (06)
Same keys as in sim.
```bash
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml             # 1
python run_controller_v2.py --config configs/franka_fr3/controllers/ik_qp_hw.yaml   # 2
python run_visualizer.py --config configs/visualizer/franka_fr3/ik_qp_hw.yaml       # 3
```

### Impedance, v2 (08)
Same keys as in sim. If it feels too stiff, lower the `kq_scale` / `dq_scale` sliders in Viser.
```bash
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml                 # 1
python run_controller_v2.py --config configs/franka_fr3/controllers/impedance_hw.yaml   # 2
python run_visualizer.py --config configs/visualizer/franka_fr3/impedance_hw.yaml       # 3
```

### Camera (RealSense): setup, once per camera / camera move
Needs `pip install pyrealsense2` (rophi env) and, for the point tracking, the TAPIR checkpoint (see README).
1. In `configs/sensors/realsense/one_realsense.yaml` set `serial` and `fx fy cx cy` of your camera (command in the
   file) and a rough `tf_link_to_cam`: measure the camera position and a point it looks at, in the robot base frame
   (x forward, y left, z up), then paste the output of
   ```bash
   python -m utils.calibration.eyeball_camera_pose --eye X Y Z --look_at X Y Z
   ```
2. Check the image: `python run_sensors.py --config configs/sensors/realsense/one_realsense.yaml --visualize`
   (the whole arm should be in view).

### Differentiable base–camera calibration (14)
Push the arm by hand (gravity compensation) to a pose where it is fully in view and still, press `SPACE` in the
**Calib RGB** window, left-click the arm in the SAM2 window, `ENTER`. Repeat for 4 different poses (`ESC` finishes
with ≥ 2). After the optimization press any key in the mosaic window. The result is written to
`configs/calibration_tutorial/results/d455_1.yaml`; the terminal also prints a `tf_link_to_cam` you can paste into the
RealSense config as the start for the next calibration. "Error vs GT" in the printout is the change from the
starting guess, not an error (there is no ground truth on the real robot).
```bash
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml                         # 1
python run_controller_v2.py --config configs/franka_fr3/controllers/grav_comp_hw.yaml           # 2
python run_sensors.py --config configs/sensors/realsense/one_realsense.yaml                     # 3
python run_camera_calibration_v2.py --config configs/calibration_tutorial/calib_diff_render_hw.yaml \
    --output configs/calibration_tutorial/results/d455_1.yaml                                   # 4
```

### Click-and-track pointing (perception + IK QP v2)
Needs the calibration above. Click a point on an object (not on the robot) in the **Click on Image** window; the
gripper (tcp) follows it, 15 cm above it, at most 0.3 m/s. Keys (terminal 3): `t` follow on/off, `h` home,
`o`/`c` gripper, `q` quit. Target jumps > 15 cm are ignored unless they persist 0.5 s; without a point for 0.5 s the
last target is held. Click again to track a new point. Quit: `q` in terminal 3 first, then Ctrl-C in 4, 2, 1.
```bash
python run_hardware.py --config configs/franka_fr3/hardware/default.yaml                                   # 1
python run_sensors.py --config configs/sensors/realsense/one_realsense.yaml                               # 2
python run_controller_v2.py --config configs/point_tracking/controllers/ik_qp_point_follow_hw.yaml       # 3
python run_perception.py --config configs/point_tracking/perception/tapnet_click_and_track_hw.yaml      # 4
python run_visualizer.py --config configs/visualizer/franka_fr3/ik_qp_hw.yaml                            # 5 (optional)
```

### Mock mode (no robot, no gripper, no camera)
Same commands as above, with `robot.mock_mode: true` in `configs/franka_fr3/hardware/bridge.yaml` (the gripper
follows it) and `mock_mode: true` in `configs/sensors/realsense/one_realsense.yaml`. The bridge then echoes the
commands as measurements, and the camera is a MuJoCo render of the cell (`mock` section) at the measured joints, with
the config's K: the robot and the target cube are where the commands put them. In the calibration nobody can push
the arm, so use `ik_qp_hw.yaml` as terminal 2 and move the arm with its keys between captures. The mock cube starts
circling `mock.target.start_delay` s after `run_sensors.py` starts. Set both back to `false` for the real robot.
