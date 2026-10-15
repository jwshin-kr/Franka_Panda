import copy

import numpy as np

import mujoco as mj

from robot_platform.sim.BaseSimPlatform import BaseSimPlatform
from data_type.basic_types.JointCtrlData import JointCtrlData
from data_type.basic_types.JointMeasData import JointMeasData
from utils.collision.fr3_robotiq_geoms import build_robot_collision_primitives
from utils.gripper import robotiq_2f85 as rq


_NUM_ARM_JOINTS = 7
_NUM_HAND_JOINTS = 1
_NUM_JOINTS = _NUM_ARM_JOINTS + _NUM_HAND_JOINTS

_ARM_JOINT_NAMES = [f"joint{i + 1}" for i in range(_NUM_ARM_JOINTS)]

# Robotiq 2F-85: one DoF, the right driver angle (0 = open .. 0.8 rad =
# closed). The left driver and the linkage follow through the MJCF equality
# constraints. Driven by the gripper's own position servo (actuator_gripper).
_HAND_JOINT_NAMES = ["robotiq_right_driver_joint"]
_GRIPPER_ACTUATOR = "actuator_gripper"


class SimFrankaFR3(BaseSimPlatform):
    """MuJoCo platform for the 7-DoF Franka Research 3 arm + Robotiq 2F-85
    gripper (the DROID setup).

    Same structure as SimFlexivArm5FHand (one 8-DoF command in, two
    measurement channels out): the arm is driven by torque (+ optional plant
    gravity comp). The gripper, like the real 2F-85, is position controlled:
    its q_des (driver angle, rad) is the set-point of the gripper's position
    servo; its qd_des / tau_ff / kp / kd are ignored.

    Command / measurement layout is [arm(7), gripper(1)] = 8. The gripper
    measurement is the right driver angle / velocity / drive force.
    """

    def __init__(
        self,
        *args,
        arm_joint_meas_freq: int = 1000,
        hand_joint_meas_freq: int = 1000,
        default_q_values=None,
        arm_joint_meas_channel: str = "sw_franka_arm_joint_meas",
        hand_joint_meas_channel: str = "sw_franka_gripper_joint_meas",
        gravity_comp: bool = True,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)

        if default_q_values is None:
            default_q_values = [0.0] * _NUM_JOINTS
        if len(default_q_values) != _NUM_JOINTS:
            raise ValueError(
                f"default_q_values must have {_NUM_JOINTS} entries "
                f"([arm(7), gripper(1)]), got {len(default_q_values)}."
            )

        self.arm_joint_name = list(_ARM_JOINT_NAMES)
        self.hand_joint_name = list(_HAND_JOINT_NAMES)

        self.arm_joint_meas_data = JointMeasData(num_joints=_NUM_ARM_JOINTS)
        self.hand_joint_meas_data = JointMeasData(num_joints=_NUM_HAND_JOINTS)

        self._default_q = np.array(default_q_values, dtype=np.float64)
        self._last_ctrl_data = JointCtrlData(num_joints=_NUM_JOINTS)

        self._gravity_comp = gravity_comp

        # Gates the startup-only arm gravity-comp fallback (see apply_sim_control).
        # Flipped True the first time a real controller command arrives.
        self._ever_received_ctrl = False

        self._arm_joint_meas_channel = arm_joint_meas_channel
        self._hand_joint_meas_channel = hand_joint_meas_channel

        self._list_of_arm_joint_indices = []
        self._list_of_hand_joint_indices = []
        self._list_of_joint_indices = []
        self._list_of_arm_actuator_indices = []
        self._gripper_actuator_idx = None

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
        model = mj_data.model
        # All robot joints are 1-DoF hinges, but go through the address tables
        # anyway: the gripper linkage puts 8 joints after the arm.
        self._arm_qpos_adr = [model.jnt_qposadr[j] for j in self._list_of_arm_joint_indices]
        self._arm_dof_adr = [model.jnt_dofadr[j] for j in self._list_of_arm_joint_indices]
        self._hand_qpos_adr = [model.jnt_qposadr[j] for j in self._list_of_hand_joint_indices]
        self._hand_dof_adr = [model.jnt_dofadr[j] for j in self._list_of_hand_joint_indices]
        self._list_of_arm_actuator_indices = [
            mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, f"motor_{n}")
            for n in self.arm_joint_name
        ]
        self._gripper_actuator_idx = mj.mj_name2id(
            model, mj.mjtObj.mjOBJ_ACTUATOR, _GRIPPER_ACTUATOR
        )
        self._gripper_ctrl_range = model.actuator_ctrlrange[self._gripper_actuator_idx].copy()

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

        t, q_des, qd_des, tau_ff, kp, kd = ctrl_data.get_data()
        self._last_ctrl_data.set_data(t, q_des, qd_des, tau_ff, kp, kd)

        arm = slice(0, _NUM_ARM_JOINTS)
        tau_command = (
            kp[arm] * (q_des[arm] - arm_q) + kd[arm] * (qd_des[arm] - arm_qd) + tau_ff[arm]
        )

        if self._gravity_comp or not self._ever_received_ctrl:
            # Gravity/Coriolis feed-forward on the ARM only (a zero command then
            # holds pose, like the real arm's built-in gravity comp). This
            # includes the gripper's load on the arm.
            tau_command = tau_command + mj_data.qfrc_bias[self._arm_dof_adr]

        mj_data.ctrl[self._list_of_arm_actuator_indices] = tau_command
        # Gripper: position set-point (driver angle) of its own servo.
        mj_data.ctrl[self._gripper_actuator_idx] = np.clip(
            q_des[_NUM_ARM_JOINTS], *self._gripper_ctrl_range
        )

    def sync_intr_data_from_sim(self, mj_data: mj.MjData, intr_pub_que_dict: dict):
        t = mj_data.time

        # [Arm] accumulator-deadline gating so the long-term publish rate matches
        # pub_dt exactly.
        arm_ch = self._arm_joint_meas_channel
        if t + 1e-9 >= self._last_pub_time[arm_ch] + self._pub_dt[arm_ch]:
            arm_q = mj_data.qpos[self._arm_qpos_adr]
            arm_qd = mj_data.qvel[self._arm_dof_adr]
            arm_tau = mj_data.qfrc_actuator[self._arm_dof_adr]
            self.arm_joint_meas_data.set_data(t, arm_q, arm_qd, arm_tau)
            intr_pub_que_dict[arm_ch].put(copy.deepcopy(self.arm_joint_meas_data))
            self._last_pub_time[arm_ch] = (
                self._last_pub_time[arm_ch] + self._pub_dt[arm_ch]
            )

        # [Gripper] driver angle / velocity / drive torque on the right driver
        hand_ch = self._hand_joint_meas_channel
        if t + 1e-9 >= self._last_pub_time[hand_ch] + self._pub_dt[hand_ch]:
            hand_q = mj_data.qpos[self._hand_qpos_adr]
            hand_qd = mj_data.qvel[self._hand_dof_adr]
            hand_tau = mj_data.qfrc_actuator[self._hand_dof_adr]
            self.hand_joint_meas_data.set_data(t, hand_q, hand_qd, hand_tau)
            intr_pub_que_dict[hand_ch].put(copy.deepcopy(self.hand_joint_meas_data))
            self._last_pub_time[hand_ch] = (
                self._last_pub_time[hand_ch] + self._pub_dt[hand_ch]
            )

    def reset(self, mj_data: mj.MjData):
        # Seed the "last command" with a light arm PD at the default pose and
        # the default gripper opening, so the robot stays put if the env runs
        # before/without a controller (ctrl_data is None -> this is applied).
        # Any real controller command immediately overrides this.
        hold_kp = np.zeros(_NUM_JOINTS)
        hold_kd = np.zeros(_NUM_JOINTS)
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

        model = mj_data.model
        mj_data.qpos[self._arm_qpos_adr] = self._default_q[:_NUM_ARM_JOINTS]
        mj_data.qvel[self._arm_dof_adr] = np.zeros((_NUM_ARM_JOINTS,))
        # Put the whole closed linkage at the default driver angle, so the
        # equality constraints start satisfied.
        driver = float(self._default_q[_NUM_ARM_JOINTS])
        for name, k in rq.MJ_LINKAGE.items():
            j = model.joint(name)
            mj_data.qpos[j.qposadr[0]] = k * driver
            mj_data.qvel[j.dofadr[0]] = 0.0
        mj_data.qfrc_actuator[:] = 0.0
        mj_data.ctrl[self._list_of_arm_actuator_indices] = np.zeros((_NUM_ARM_JOINTS,))
        mj_data.ctrl[self._gripper_actuator_idx] = driver

        self._last_pub_time[self._arm_joint_meas_channel] = mj_data.time
        self._last_pub_time[self._hand_joint_meas_channel] = mj_data.time

    def get_col_info_data(self):
        """Robot collision primitives (per arm link + gripper), in each link's
        local frame. Consumed by the env, published on robot_col_info, and used
        by the collision-aware controller for FCL distance queries."""
        return build_robot_collision_primitives()
