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
