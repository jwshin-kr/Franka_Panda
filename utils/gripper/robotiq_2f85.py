"""Robotiq 2F-85 (the DROID gripper): ranges and unit conversions shared by the
sim, the hardware driver and the configs.

The gripper has one DoF. Everywhere in this repo (joint_ctrl_t / joint_meas_t
slot 7, MJCF, URDF) it is the **driver angle** ``robotiq_right_driver_joint``
in rad, 0 = open .. ~0.8 = closed; the rest of the four-bar linkage follows
it. The real gripper is position controlled over Modbus with a **register
position** 0 (open) .. 255 (closed); ``hardware/franka_fr3/Robotiq2F85Hardware``
converts between the two through the pad **width** (distance between the two
pad faces, m), which is what the register position is linear in.

The angle -> width table is a sweep of ``assets/scene/franka_fr3/fr3_robot.xml``;
the last row is extrapolated to touching pads.
"""

import numpy as np

MAX_WIDTH = 0.085  # m, fully open (the model opens to 85.2 mm)
MAX_ANGLE = 0.8  # rad, driver joint range (MJCF / URDF)
SPEED_RANGE = (0.02, 0.15)  # m/s finger speed, register 0..255 (datasheet)
FORCE_RANGE = (20.0, 235.0)  # N grip force, register 0..255 (datasheet)
MASS = 0.925  # kg incl. the coupling (datasheet)

# driver angle (rad) -> pad-face width (m)
_ANGLE = np.array([0.0, 0.0526, 0.1025, 0.1525, 0.2024, 0.2524, 0.3023, 0.3523, 0.4022,
                   0.4522, 0.5021, 0.5521, 0.602, 0.652, 0.7019, 0.7519, 0.785])
_WIDTH = np.array([0.0853, 0.0806, 0.0758, 0.0709, 0.0658, 0.0606, 0.0553, 0.0499, 0.0443,
                   0.0387, 0.033, 0.0272, 0.0214, 0.0155, 0.0097, 0.0038, 0.0])
CLOSED_ANGLE = float(_ANGLE[-1])


def width_from_angle(angle):
    """Driver angle (rad) -> pad-face width (m)."""
    return np.interp(angle, _ANGLE, _WIDTH)


def angle_from_width(width):
    """Pad-face width (m) -> driver angle (rad)."""
    return np.interp(width, _WIDTH[::-1], _ANGLE[::-1])


# MuJoCo linkage joints as multiples of the driver angle with nothing in the
# hand (the <mimic> multipliers of urdf/fr3_robotiq_viser.urdf; couplers ~0).
# Used to put the closed linkage at an opening on reset.
MJ_LINKAGE = {
    f"robotiq_{s}_{j}_joint": k
    for s in ("right", "left")
    for j, k in (("driver", 1.0), ("coupler", 0.0), ("spring_link", 1.0), ("follower", -0.963))
}
