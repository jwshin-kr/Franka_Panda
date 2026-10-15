"""Robot collision primitives of the Franka Research 3 + Robotiq 2F-85, one per link
name, in each link's local frame (the controller left-multiplies by the link's
world pose from FK). Shared by the sim (SimFrankaFR3) and the real robot
(HardwareFrankaFR3), so the collision-aware controllers get the same
geometry from both. Returns the list of dicts ColInfoData.add_col_geom takes.
"""

import numpy as np

# One cylinder per arm link, as (center in the link frame, axis, radius, total
# cylinder length). Derived from the FR3 self-collision capsules of
# franka_description (Franka Robotics, Apache-2.0; fr3 xacro with_sc,
# safety_distance 0; each cylinder encloses its capsule: length = segment +
# 2 radius). ColInfo carries one primitive per link name, so where
# franka_description has a second capsule (link5, link7) the main one is
# widened instead: link5 r 0.09 -> 0.095, link7 r 0.07 -> 0.091 spanning
# z -0.13 .. 0.11. Checked on fr3_robot.xml: together they enclose every
# arm collision-mesh vertex at 300 random poses. Frame names (link1..link7)
# must match the pinocchio BODY frames the collision-aware controller does
# FK on.
_ARM_LINK_CYLINDERS = {
    "link1": ([0.0, 0.0, -0.1915], [0.0, 0.0, 1.0], 0.09, 0.283 + 2 * 0.09),
    "link2": ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.09, 0.12 + 2 * 0.09),
    "link3": ([0.0, 0.0, -0.145], [0.0, 0.0, 1.0], 0.09, 0.15 + 2 * 0.09),
    "link4": ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.09, 0.12 + 2 * 0.09),
    "link5": ([0.0, 0.0, -0.26], [0.0, 0.0, 1.0], 0.095, 0.10 + 2 * 0.095),
    "link6": ([0.0, 0.0, -0.03], [0.0, 0.0, 1.0], 0.08, 0.08 + 2 * 0.08),
    "link7": ([0.0, 0.0, -0.01], [0.0, 0.0, 1.0], 0.091, 0.24),
}

# Robotiq 2F-85: one box in the "gripper" frame (= flange) enclosing every
# collision geom of the gripper (coupling, base, linkage, pads) over the whole
# stroke, from a sweep of fr3_robot.xml: x +-37.5 mm, y +-74.2 mm (fingers
# open, closing axis), z 0 .. 173.7 mm (approach axis).
_GRIPPER_BOX_SIZE = [0.075, 0.149, 0.174]  # full extents
_GRIPPER_BOX_CENTER = [0.0, 0.0, 0.087]


def _align_z_to(u: np.ndarray) -> np.ndarray:
    """Rotation whose +z axis points along unit-ish vector u (for FCL cylinders,
    which are defined along local z)."""
    u = u / np.linalg.norm(u)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, u)
    c = float(z @ u)
    if np.linalg.norm(v) < 1e-8:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def build_gripper_collision_primitives():
    """The Robotiq 2F-85 as a single box on the "gripper" frame."""
    offset = np.eye(4)
    offset[:3, 3] = _GRIPPER_BOX_CENTER
    return [
        {
            "name": "gripper",
            "geom_type": "box",
            "size": np.array(_GRIPPER_BOX_SIZE),
            "offset": offset,
        }
    ]


def build_robot_collision_primitives():
    """Per-link collision primitives for the arm + gripper: one cylinder per
    arm link (table above, link-local frame) and the gripper box.

    Offsets are LINK-LOCAL SE3, matching SimFlexivArm5FHand.get_col_info_data.
    """
    prims = []
    for link, (center, axis, radius, length) in _ARM_LINK_CYLINDERS.items():
        offset = np.eye(4)
        offset[:3, :3] = _align_z_to(np.asarray(axis, dtype=float))
        offset[:3, 3] = center
        prims.append(
            {
                "name": link,
                "geom_type": "cylinder",
                "size": np.array([radius, length]),
                "offset": offset,
            }
        )
    prims.extend(build_gripper_collision_primitives())
    return prims
