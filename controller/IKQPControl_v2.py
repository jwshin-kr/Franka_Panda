"""IK QP controller, v2: tutorial 05 cause 1 (stale LCM data) fixed for tutorial 06.

The original controller (``IKQPControl.py``) is left untouched.

Cause: stale data in both directions (same as tutorial 05)
    * joint_meas in: one message per channel is popped per loop iteration from
      an unbounded FIFO, so the start-up backlog never drains.
    * joint_ctrl out: ``ctrl_pub_que`` is an unbounded FIFO emptied one command
      per publisher-thread loop.
    Measured with the original after ~20 s: the joint state used was ~1 s old
    and the command the env applied ~3.6 s old, both growing linearly.
Fix (same as ``CollisionAwareGravCompControl_v2``): drain the measurement
queues (newest sample only) and keep only the newest unsent command.

Also: no torque steps from the Coulomb friction feedforward. With the FR3's
joint friction (Fjc ~1.1 Nm) the term Fjc * sat(qd_des / friction_phi) jumps
between 0 and +-Fjc whenever qd_des is zeroed (target reached, pause) or
changes sign, i.e. ~1 Nm within one 1 ms tick: more than libfranka's
1000 Nm/s torque-rate limit. Its change is limited to ``friction_rate_max``
(Nm/s); gravity / Coriolis are not delayed.

Also (``track_reference``): exact convergence to the target. The original
tracks only the QP's joint velocity (kp = 0, kd = velocity_control_D_gain) and,
inside ``pos_tol`` (5 mm), holds q_des = q, the *measured* pose with a weak
spring. With real joint friction the tip stops short of the target, and the
hold follows the arm wherever friction or a small gravity-model error moves it.
Same control law (contracting field -> IK QP -> joint torque), but the QP's
qd_des is integrated into a joint reference q_ref and tracked by a stiff joint
impedance:

    q_ref += qd_des * dt          (|q_ref - q| <= ref_max_error, anti-windup)
    tau    = ref_kp (q_ref - q) + ref_kd (qd_des - qd) + g(q) + C qd + friction

While the measured tip is off target the Cartesian field keeps advancing q_ref,
so the torque grows until friction gives way (integral-like action); at the
target q_ref stops and the arm is held exactly there. ``ref_pos_tol``,
``ref_vel_gain`` and ``ref_v_max`` replace pos_tol, vel_gain and v_max; qd_des
is acceleration limited (``ref_qd_max_acc``) so the stiff joint impedance gets
no steps. q_ref starts from (and re-syncs to) the measured pose: before the
first arm measurement no command is sent, and after a pause / gap of more than
0.1 s q_ref restarts at the measured pose.
"""

from __future__ import annotations

from queue import Empty

import numpy as np

from controller.CollisionAwareGravCompControl_v2 import _LatestOnlyQueue
from controller.IKQPControl import IKQPController
from data_type.basic_types.JointCtrlData import JointCtrlData
from data_type.basic_types.JointMeasData import JointMeasData


class IKQPControllerV2(IKQPController):
    def __init__(self, config, *args, **kwargs):
        super().__init__(config, *args, **kwargs)
        self._friction_rate_max = float(config.get("friction_rate_max", 100.0))
        self._friction_prev = np.zeros(self._n_x, dtype=np.float64)

        self._track_reference = bool(config.get("track_reference", False))
        if self._track_reference:
            self._ref_kp = np.asarray(config["ref_kp"], dtype=np.float64)
            self._ref_kd = np.asarray(config["ref_kd"], dtype=np.float64)
            self._ref_max_error = float(config.get("ref_max_error", 0.05))
            self._ref_qd_max_acc = float(config.get("ref_qd_max_acc", 5.0))
            self._pos_tol = float(config.get("ref_pos_tol", self._pos_tol))
            self._vel_gain = float(config.get("ref_vel_gain", self._vel_gain))
            self._v_max = float(config.get("ref_v_max", self._v_max))
            print(f"[IKQP v2] Joint reference tracking: kp={self._ref_kp}, "
                  f"kd={self._ref_kd}, pos_tol={self._pos_tol * 1000:.1f} mm, "
                  f"vel_gain={self._vel_gain}, v_max={self._v_max} m/s")
        self._q_ref = None
        self._qd_ref = None
        self._last_update_t = None
        self._arm_meas_channel = self._dict_joints["arm"]["joint_meas_channel"]
        self._have_arm_meas = False

    def _friction_compensation(self, qd_des: np.ndarray) -> np.ndarray:
        target = super()._friction_compensation(qd_des)
        max_step = self._friction_rate_max * self._ctrl_dt
        self._friction_prev = self._friction_prev + np.clip(
            target - self._friction_prev, -max_step, max_step
        )
        return self._friction_prev.copy()

    def _check_and_get_data_from_que(self) -> None:
        for joint_cfg in self._dict_joints.values():
            channel = joint_cfg["joint_meas_channel"]
            idx = joint_cfg["list_joint_idx"]
            meas = None
            while True:  # drain -> newest sample only
                try:
                    meas = self.intr_sub_que_dict[channel].get_nowait()
                except Empty:
                    break
            if meas is None:
                continue
            if not isinstance(meas, JointMeasData):
                raise ValueError(f"Expected JointMeasData on '{channel}'")
            _, q, qd, _ = meas.get_data()
            with self._meas_lock:
                self._q[idx] = q
                self._qd[idx] = qd
            if channel == self._arm_meas_channel:
                self._have_arm_meas = True

    def _update(self) -> None:
        if not self._track_reference:
            super()._update()
            return
        if not self._have_arm_meas:
            return  # no command before the real arm pose is known

        self._consume_latest_plan()
        with self._meas_lock:
            q = self._q.copy()
            qd = self._qd.copy()

        # (re)start the reference at the measured pose
        gap = self._last_update_t is None or self._cur_time - self._last_update_t > 0.1
        self._last_update_t = self._cur_time
        if self._q_ref is None or gap:
            self._q_ref = q.copy()
            self._qd_des_filt[:] = 0.0

        # same low-pass on the QP's joint velocity as the original
        a = float(np.clip(self._qd_filter, 0.0, 1.0))
        self._qd_des_filt = a * self._qd_des + (1.0 - a) * self._qd_des_filt
        target = self._qd_des_filt
        if self._at_goal or not np.any(np.abs(self._qd_des) > 1e-9):
            target = np.zeros_like(target)  # at the target: q_ref stops, arm held there
        # acceleration limit: the 100 Hz plan steps reach the stiff joint
        # impedance smoothly (no torque-rate spikes)
        max_dv = self._ref_qd_max_acc * self._ctrl_dt
        if gap or self._qd_ref is None:
            self._qd_ref = np.zeros_like(target)
        self._qd_ref = self._qd_ref + np.clip(target - self._qd_ref, -max_dv, max_dv)
        qd_des = self._qd_ref.copy()

        q_ref = self._q_ref + qd_des * self._ctrl_dt
        q_ref = q + np.clip(q_ref - q, -self._ref_max_error, self._ref_max_error)
        with self._meas_lock:
            self._q_ref = np.clip(q_ref, self._q_lower, self._q_upper)

        tau_ff = self._tau_ff(q, qd, qd_des)
        q_des = self._with_gripper_set_point(self._q_ref)
        cmd = JointCtrlData(num_joints=self._num_joints)
        cmd.set_data(self._cur_time, q_des, qd_des, tau_ff, self._ref_kp, self._ref_kd)
        self.ctrl_pub_que.put(cmd)
        self._publish_target(tip=self._tip, velocity=self._v_cmd)

    def set_ctrl_pub_que(self, pub_que) -> None:
        """Unsent stale commands are dropped on put()."""
        super().set_ctrl_pub_que(_LatestOnlyQueue(pub_que))
