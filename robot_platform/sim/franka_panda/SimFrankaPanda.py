import copy

import numpy as np

import mujoco as mj

from robot_platform.sim.BaseSimPlatform import BaseSimPlatform
from data_type.basic_types.JointCtrlData import JointCtrlData
from data_type.basic_types.JointMeasData import JointMeasData


_NUM_ARM_JOINTS = 7
_NUM_HAND_JOINTS = 2
_NUM_JOINTS = _NUM_ARM_JOINTS + _NUM_HAND_JOINTS

# Panda self-collision capsules from franka_description (Franka Robotics,
# Apache-2.0), robots/common/franka_arm.xacro with safety_distance = 0, as
# (center in the link frame, capsule axis, radius, cylinder length between the
# two cap centers). Frame names (link1..link7, hand) must match the pinocchio
# BODY frames the collision-aware controller does FK on. Where
# franka_description lists two capsules for a link (link5, link7) the larger
# one is kept: ColInfo carries one primitive per link name.
_ARM_LINK_CAPSULES = {
    "link1": ([0.0, 0.0, -0.1915], [0.0, 0.0, 1.0], 0.06, 0.283),
    "link2": ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.06, 0.12),
    "link3": ([0.0, 0.0, -0.145], [0.0, 0.0, 1.0], 0.06, 0.15),
    "link4": ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.06, 0.12),
    "link5": ([0.0, 0.0, -0.26], [0.0, 0.0, 1.0], 0.06, 0.10),
    "link6": ([0.0, 0.0, -0.03], [0.0, 0.0, 1.0], 0.05, 0.08),
    "link7": ([0.0, 0.0, 0.01], [0.0, 0.0, 1.0], 0.04, 0.14),
}


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


def _build_hand_collision_primitives():
    """Collision primitives for the Franka Hand: the hand body (the larger
    franka_description hand capsule, axis along the finger travel direction y)
    + one box per finger (bounding box of the finger collision mesh, in the
    finger frame; both fingers use the same mesh in their own frame)."""
    finger_offset = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0144],
            [0.0, 0.0, 1.0, 0.027],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    return [
        {
            "name": "hand",
            "geom_type": "cylinder",
            # capsule radius 0.04, length 0.1 -> cylinder enclosing the caps
            "size": np.array([0.04, 0.1 + 2 * 0.04]),
            "offset": np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, -1.0, 0.0, 0.04],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        },
        {
            "name": "left_finger",
            "geom_type": "box",
            "size": np.array([0.021, 0.024, 0.054]),
            "offset": finger_offset.copy(),
        },
        {
            "name": "right_finger",
            "geom_type": "box",
            "size": np.array([0.021, 0.024, 0.054]),
            "offset": finger_offset.copy(),
        },
    ]


def _build_arm_collision_primitives():
    """Per-link collision primitives for the arm + hand. Each arm link is a
    cylinder enclosing its franka_description capsule (cylinder length + both
    hemispherical caps), placed at the capsule center along the capsule axis in
    the link-local frame; the hand is the primitive set above.
    Returns the list of dicts consumed by ColInfoData.add_col_geom.

    Offsets are LINK-LOCAL SE3 (the controller left-multiplies by the link's
    world pose from FK), matching SimFlexivArm5FHand.get_col_info_data.
    """
    prims = []
    for link, (center, axis, radius, length) in _ARM_LINK_CAPSULES.items():
        offset = np.eye(4)
        offset[:3, :3] = _align_z_to(np.asarray(axis, dtype=float))
        offset[:3, 3] = center  # cylinder center = capsule center
        prims.append(
            {
                "name": link,
                "geom_type": "cylinder",
                "size": np.array([radius, length + 2.0 * radius]),
                "offset": offset,
            }
        )
    # Hand: hand body + one box per finger.
    prims.extend(_build_hand_collision_primitives())
    return prims


_ARM_JOINT_NAMES = [f"joint{i + 1}" for i in range(_NUM_ARM_JOINTS)]

# Franka Hand: two prismatic fingers, each 0 (closed) .. 0.04 m (open), in the
# order panda_robot.xml declares them.
_HAND_JOINT_NAMES = ["finger_joint1", "finger_joint2"]


class SimFrankaPanda(BaseSimPlatform):
    """MuJoCo platform for the 7-DoF Franka Emika Panda arm + Franka Hand.

    Same structure as SimFlexivArm5FHand (one 9-DoF command in, two
    measurement channels out): the arm is driven by torque (+ optional plant
    gravity comp) and the two fingers by joint-space PD.

    Command / measurement layout is [arm(7), hand(2)] = 9, matching the
    panda_robot.xml joint order.
    """

    def __init__(
        self,
        *args,
        arm_joint_meas_freq: int = 1000,
        hand_joint_meas_freq: int = 1000,
        default_q_values=None,
        arm_joint_meas_channel: str = "sw_franka_arm_joint_meas",
        hand_joint_meas_channel: str = "sw_franka_hand_joint_meas",
        gravity_comp: bool = True,
        default_hand_hold_kp: float = 200.0,
        default_hand_hold_kd: float = 5.0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)

        if default_q_values is None:
            default_q_values = [0.0] * _NUM_JOINTS
        if len(default_q_values) != _NUM_JOINTS:
            raise ValueError(
                f"default_q_values must have {_NUM_JOINTS} entries "
                f"([arm(7), hand(2)]), got {len(default_q_values)}."
            )

        self.arm_joint_name = list(_ARM_JOINT_NAMES)
        self.hand_joint_name = list(_HAND_JOINT_NAMES)

        self.arm_joint_meas_data = JointMeasData(num_joints=_NUM_ARM_JOINTS)
        self.hand_joint_meas_data = JointMeasData(num_joints=_NUM_HAND_JOINTS)

        self._default_q = np.array(default_q_values, dtype=np.float64)
        self._last_ctrl_data = JointCtrlData(num_joints=_NUM_JOINTS)

        self._gravity_comp = gravity_comp
        # Finger hold gains are N/m and N/(m/s) (prismatic joints).
        self._default_hand_hold_kp = default_hand_hold_kp
        self._default_hand_hold_kd = default_hand_hold_kd

        # Gates the startup-only arm gravity-comp fallback (see apply_sim_control).
        # Flipped True the first time a real controller command arrives.
        self._ever_received_ctrl = False

        self._arm_joint_meas_channel = arm_joint_meas_channel
        self._hand_joint_meas_channel = hand_joint_meas_channel

        self._list_of_arm_joint_indices = []
        self._list_of_hand_joint_indices = []
        self._list_of_joint_indices = []

        self._last_pub_time = {
            self._arm_joint_meas_channel: 0.0,
            self._hand_joint_meas_channel: 0.0,
        }
        self._pub_dt = {
            self._arm_joint_meas_channel: 1.0 / arm_joint_meas_freq,
            self._hand_joint_meas_channel: 1.0 / hand_joint_meas_freq,
        }

    def set_mj_data_name_idx(self, mj_data: mj.MjData):
        for joint_name in self.arm_joint_name:
            self._list_of_arm_joint_indices.append(
                mj.mj_name2id(mj_data.model, mj.mjtObj.mjOBJ_JOINT, joint_name)
            )
        for joint_name in self.hand_joint_name:
            self._list_of_hand_joint_indices.append(
                mj.mj_name2id(mj_data.model, mj.mjtObj.mjOBJ_JOINT, joint_name)
            )
        self._list_of_joint_indices = (
            self._list_of_arm_joint_indices + self._list_of_hand_joint_indices
        )

    def apply_sim_control(self, ctrl_data: JointCtrlData, mj_data: mj.MjData):
        # Startup-only safety hold. Before the FIRST real controller command
        # arrives, fall back to the seeded hold + arm gravity comp: this env runs
        # with gravity_comp=False, so a raw zero/PD command does NOT hold the arm
        # (joint2/joint4 need ~30/~22 Nm at the default pose) and it would fall
        # before the controller connects.
        #
        # Once a controller has connected, we must NOT keep adding gravity comp on
        # empty-queue ticks: the held _last_ctrl_data already carries the
        # controller's OWN tau_ff = g(q), so adding qfrc_bias on top would be
        # double gravity comp and send the arm shooting upward. So the fallback
        # comp is gated on "never received a real command yet", not on this tick
        # being empty.
        if ctrl_data is None:
            ctrl_data = self._last_ctrl_data
        else:
            self._ever_received_ctrl = True

        _, arm_q, arm_qd, _ = self.arm_joint_meas_data.get_data()
        _, hand_q, hand_qd, _ = self.hand_joint_meas_data.get_data()
        q = np.concatenate((arm_q, hand_q))
        qd = np.concatenate((arm_qd, hand_qd))

        t, q_des, qd_des, tau_ff, kp, kd = ctrl_data.get_data()
        self._last_ctrl_data.set_data(t, q_des, qd_des, tau_ff, kp, kd)

        tau_command = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff

        if self._gravity_comp or not self._ever_received_ctrl:
            # Gravity/Coriolis feed-forward on the ARM only (a zero command then
            # holds pose, like the real arm's built-in gravity comp). The light
            # fingers are held by their PD gains.
            arm_slice = slice(0, _NUM_ARM_JOINTS)
            tau_command[arm_slice] = tau_command[arm_slice] + (
                mj_data.qfrc_bias[self._list_of_arm_joint_indices]
            )

        mj_data.ctrl[self._list_of_joint_indices] = tau_command

    def sync_intr_data_from_sim(self, mj_data: mj.MjData, intr_pub_que_dict: dict):
        t = mj_data.time

        # [Arm] accumulator-deadline gating so the long-term publish rate matches
        # pub_dt exactly.
        arm_ch = self._arm_joint_meas_channel
        if t + 1e-9 >= self._last_pub_time[arm_ch] + self._pub_dt[arm_ch]:
            arm_q = mj_data.qpos[self._list_of_arm_joint_indices]
            arm_qd = mj_data.qvel[self._list_of_arm_joint_indices]
            arm_tau = mj_data.qfrc_actuator[self._list_of_arm_joint_indices]
            self.arm_joint_meas_data.set_data(t, arm_q, arm_qd, arm_tau)
            intr_pub_que_dict[arm_ch].put(copy.deepcopy(self.arm_joint_meas_data))
            self._last_pub_time[arm_ch] = (
                self._last_pub_time[arm_ch] + self._pub_dt[arm_ch]
            )

        # [Hand]
        hand_ch = self._hand_joint_meas_channel
        if t + 1e-9 >= self._last_pub_time[hand_ch] + self._pub_dt[hand_ch]:
            hand_q = mj_data.qpos[self._list_of_hand_joint_indices]
            hand_qd = mj_data.qvel[self._list_of_hand_joint_indices]
            hand_tau = mj_data.qfrc_actuator[self._list_of_hand_joint_indices]
            self.hand_joint_meas_data.set_data(t, hand_q, hand_qd, hand_tau)
            intr_pub_que_dict[hand_ch].put(copy.deepcopy(self.hand_joint_meas_data))
            self._last_pub_time[hand_ch] = (
                self._last_pub_time[hand_ch] + self._pub_dt[hand_ch]
            )

    def reset(self, mj_data: mj.MjData):
        # Seed the "last command" with a finger hold (and a light arm PD) so the
        # robot stays put if the env runs before/without a controller
        # (ctrl_data is None -> this is applied). Any real controller command
        # immediately overrides this.
        hold_kp = np.zeros(_NUM_JOINTS)
        hold_kd = np.zeros(_NUM_JOINTS)
        hold_kp[_NUM_ARM_JOINTS:] = self._default_hand_hold_kp
        hold_kd[_NUM_ARM_JOINTS:] = self._default_hand_hold_kd
        # Light arm PD to keep it centered at the default pose during the startup
        # gap. Gravity comp (added in apply_sim_control when no controller is
        # driving) carries the load; this just kills drift from that marginally-
        # stable equilibrium.
        hold_kp[:_NUM_ARM_JOINTS] = 40.0
        hold_kd[:_NUM_ARM_JOINTS] = 4.0

        # Re-arm the startup gravity-comp fallback for this run.
        self._ever_received_ctrl = False
        self._last_ctrl_data.set_data(
            mj_data.time,
            self._default_q,
            np.zeros(_NUM_JOINTS),
            np.zeros(_NUM_JOINTS),
            hold_kp,
            hold_kd,
        )
        self.arm_joint_meas_data.set_q(self._default_q[:_NUM_ARM_JOINTS])
        self.hand_joint_meas_data.set_q(self._default_q[_NUM_ARM_JOINTS:])

        mj_data.qpos[self._list_of_joint_indices] = self._default_q
        mj_data.qvel[self._list_of_joint_indices] = np.zeros((_NUM_JOINTS,))
        mj_data.qfrc_actuator[self._list_of_joint_indices] = np.zeros((_NUM_JOINTS,))
        mj_data.ctrl[self._list_of_joint_indices] = np.zeros((_NUM_JOINTS,))

        self._last_pub_time[self._arm_joint_meas_channel] = mj_data.time
        self._last_pub_time[self._hand_joint_meas_channel] = mj_data.time

    def get_col_info_data(self):
        """Robot collision primitives (per arm link + hand), in each link's
        local frame. Consumed by the env, published on robot_col_info, and used
        by the collision-aware controller for FCL distance queries."""
        return _build_arm_collision_primitives()
