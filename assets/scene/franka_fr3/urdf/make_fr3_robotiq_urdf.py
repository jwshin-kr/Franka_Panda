"""Write fr3_robotiq.urdf and fr3_robotiq_viser.urdf from the MuJoCo model.

    python assets/scene/franka_fr3/urdf/make_fr3_robotiq_urdf.py

The whole robot (FR3 arm link0..link7, flange, Robotiq 2F-85 from body
``gripper`` on) is generated from ../fr3_robot.xml, so MuJoCo, Pinocchio and
Viser share one geometry and one set of inertias; the world link and the robot
cell (enclosure) come from ``_world_cell.urdf.in``. Rerun after an edit of the
XML (e.g. the gripper's mounting angle).

The 2F-85 is a closed four-bar linkage, which URDF cannot express. Its one DoF
is the right driver angle ``robotiq_right_driver_joint`` (0 = open .. 0.8 rad =
closed), the 8th joint of both files:

* fr3_robotiq.urdf (controllers, Pinocchio): the linkage is frozen at the
  open pose; only the right driver (+ coupler, 23 g) turns with the joint.
  Pinocchio 3.4 has no <mimic> support, and the gripper is position
  controlled anyway, so this keeps ``model.nq = 8`` and the arm's g(q) with
  the full 0.925 kg gripper.
* fr3_robotiq_viser.urdf (Viser, yourdfpy): the rest of the linkage follows
  the driver through <mimic> (multipliers fitted from a MuJoCo sweep,
  residual < 6 mrad; the couplers barely move and stay fixed).
"""

import os

import mujoco as mj
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "..", "fr3_robot.xml")
CELL = os.path.join(HERE, "_world_cell.urdf.in")

DRIVER = "robotiq_right_driver_joint"
# Linkage joint angle = multiplier * driver angle (no object in the hand).
MIMIC = {
    "robotiq_left_driver_joint": 1.0,
    "robotiq_right_spring_link_joint": 0.9996,
    "robotiq_left_spring_link_joint": 0.9997,
    "robotiq_right_follower_joint": -0.9628,
    "robotiq_left_follower_joint": -0.9626,
}
ARM_JOINTS = [f"joint{i + 1}" for i in range(7)]
# FR3 joint velocity limits (rad/s, datasheet); efforts come from the MJCF
# actuatorfrcrange.
ARM_VEL_LIMITS = dict(zip(ARM_JOINTS, [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]))


def fmt(v):
    return " ".join(f"{(x if abs(x) > 1e-12 else 0.0):.9g}" for x in v)


def rpy(R):
    # URDF rpy: R = Rz(y) Ry(p) Rx(r)
    p = np.arcsin(-np.clip(R[2, 0], -1, 1))
    if abs(np.cos(p)) > 1e-9:
        r, y = np.arctan2(R[2, 1], R[2, 2]), np.arctan2(R[1, 0], R[0, 0])
    else:
        r, y = np.arctan2(-R[1, 2], R[1, 1]), 0.0
    return np.array([r, p, y]) + 0.0  # + 0.0: no "-0"


def tf(pos, quat):
    T = np.eye(4)
    m = np.zeros(9)
    mj.mju_quat2Mat(m, np.asarray(quat, dtype=float))
    T[:3, :3] = m.reshape(3, 3)
    T[:3, 3] = pos
    return T


def origin(T):
    return f'<origin xyz="{fmt(T[:3, 3] + 0.0)}" rpy="{fmt(rpy(T[:3, :3]))}"/>'


def mesh_file(m, mid) -> str:
    """Mesh file as written in the MJCF (relative to ../), seen from here."""
    adr = m.mesh_pathadr[mid]
    return "../" + m.paths[adr:m.paths.index(b"\x00", adr)].decode()


def robot_xml(m, viser: bool) -> str:
    root = m.body("link0").id
    bodies = [b for b in range(root, m.nbody)]
    name = lambda b: m.body(b).name

    def joint_of(b):
        return m.body_jntadr[b] if m.body_jntnum[b] else -1

    # URDF link frame = MuJoCo body frame shifted to its joint position.
    shift = {}
    for b in bodies:
        j = joint_of(b)
        shift[b] = tf(m.jnt_pos[j], [1, 0, 0, 0]) if j >= 0 else np.eye(4)

    def link_xml(b):
        S_inv = np.linalg.inv(shift[b])
        out = [f'  <link name="{name(b)}">']
        if m.body_mass[b] > 0:
            Ti = S_inv @ tf(m.body_ipos[b], m.body_iquat[b])
            R = Ti[:3, :3]
            I = R @ np.diag(m.body_inertia[b]) @ R.T
            out += [
                "    <inertial>",
                f'      <origin xyz="{fmt(Ti[:3, 3])}" rpy="0 0 0"/>',
                f'      <mass value="{m.body_mass[b]:.9g}"/>',
                f'      <inertia ixx="{I[0,0]:.6g}" ixy="{I[0,1]:.6g}" ixz="{I[0,2]:.6g}" '
                f'iyy="{I[1,1]:.6g}" iyz="{I[1,2]:.6g}" izz="{I[2,2]:.6g}"/>',
                "    </inertial>",
            ]
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b:
                continue
            Tg = S_inv @ tf(m.geom_pos[g], m.geom_quat[g])
            if m.geom_type[g] == mj.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[g]
                geo = f'<mesh filename="{mesh_file(m, mid)}" scale="{fmt(m.mesh_scale[mid])}"/>'
                # compiled mesh geoms are re-centred; undo with the mesh offset
                Tg = Tg @ np.linalg.inv(tf(m.mesh_pos[mid], m.mesh_quat[mid]))
            elif m.geom_type[g] == mj.mjtGeom.mjGEOM_BOX:
                geo = f'<box size="{fmt(2 * m.geom_size[g])}"/>'
            else:
                continue
            tag = "collision" if m.geom_contype[g] or m.geom_conaffinity[g] else "visual"
            out.append(f"    <{tag}>")
            out.append(f"      {origin(Tg)}")
            out.append(f"      <geometry>{geo}</geometry>")
            if tag == "visual":
                rgba = m.geom_rgba[g] if m.geom_matid[g] < 0 else m.mat_rgba[m.geom_matid[g]]
                out.append(f'      <material name=""><color rgba="{fmt(rgba)}"/></material>')
            out.append(f"    </{tag}>")
        out.append("  </link>")
        return "\n".join(out)

    def joint_xml(b):
        p = m.body_parentid[b]
        T = np.linalg.inv(shift[p]) @ tf(m.body_pos[b], m.body_quat[b]) @ shift[b]
        j = joint_of(b)
        jname = m.joint(j).name if j >= 0 else f"{name(p)}_to_{name(b)}"
        movable = j >= 0 and (jname in ARM_JOINTS or jname == DRIVER or (viser and jname in MIMIC))
        if not movable:
            return (f'  <joint name="{jname}" type="fixed">\n    {origin(T)}\n'
                    f'    <parent link="{name(p)}"/>\n    <child link="{name(b)}"/>\n  </joint>')
        lo, hi = m.jnt_range[j]
        effort = m.jnt_actfrcrange[j][1] if m.jnt_actfrclimited[j] else 10.0
        vel = ARM_VEL_LIMITS.get(jname, 2.0)
        s = (f'  <joint name="{jname}" type="revolute">\n    {origin(T)}\n'
             f'    <parent link="{name(p)}"/>\n    <child link="{name(b)}"/>\n'
             f'    <axis xyz="{fmt(m.jnt_axis[j])}"/>\n'
             f'    <limit lower="{lo:.6g}" upper="{hi:.6g}" effort="{effort:.6g}" velocity="{vel:.6g}"/>\n')
        if jname in MIMIC:
            s += f'    <mimic joint="{DRIVER}" multiplier="{MIMIC[jname]}" offset="0"/>\n'
        return s + "  </joint>"

    T_root = tf(m.body_pos[root], m.body_quat[root])  # relative to the world
    parts = [
        '  <joint name="base_joint" type="fixed">\n'
        f"    {origin(T_root)}\n"
        '    <parent link="world"/>\n    <child link="link0"/>\n  </joint>'
    ]
    for b in bodies:
        parts.append(link_xml(b))
        if b != root:
            parts.append(joint_xml(b))
    return "\n".join(parts)


HEADER = """<?xml version="1.0" ?>
<!-- Franka Research 3 arm + Robotiq 2F-85 gripper (the DROID setup), for
     {use}. Written by make_fr3_robotiq_urdf.py from ../fr3_robot.xml (arm:
     Menagerie franka_fr3, Apache-2.0; gripper: Menagerie robotiq_2f85,
     BSD-2-Clause): same names, kinematics and inertias, so
     pin.computeGeneralizedGravity matches MuJoCo's qfrc_bias. Gripper:
     {gripper}
     Do not edit by hand: edit ../fr3_robot.xml / _world_cell.urdf.in and rerun
     make_fr3_robotiq_urdf.py.
     Actuated joint order: joint1..joint7, robotiq_right_driver_joint. -->
"""


def main():
    m = mj.MjModel.from_xml_path(XML)
    cell = open(CELL).read()
    head, tail = cell.split("  <!-- @ROBOT@ -->\n")
    for viser, fname, use, gripper in (
        (False, "fr3_robotiq.urdf", "Pinocchio (dynamics, controllers)",
         "linkage frozen open, only the\n     right driver turns."),
        (True, "fr3_robotiq_viser.urdf", "Viser (visualization)",
         "linkage follows the right driver\n     through <mimic>."),
    ):
        text = HEADER.format(use=use, gripper=gripper) + head + robot_xml(m, viser) + "\n" + tail
        open(os.path.join(HERE, fname), "w").write(text)
        print("wrote", fname)


if __name__ == "__main__":
    main()
