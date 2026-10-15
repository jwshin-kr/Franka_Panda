"""Impedance controller, v2: stale LCM data fixed for tutorial 08.

The original controller (``ImpedanceControl.py``) is left untouched.

Same as RoPhiTutorial's ``controller/ImpedanceControl_v2.py``; the FR3 changes
(task frame ``tcp``, Robotiq set-point in the gripper's q_des slot) live in
``ImpedanceControl.py``. (The measurements below are from the Flexiv arm.)

Cause: the controller acts on an old joint state (same as tutorials 05-07)
    * joint_meas in: ``_check_and_get_data_from_que`` pops ONE message per
      channel per loop iteration from an unbounded FIFO. The env publishes
      1000 per second and the loop manages slightly fewer iterations, so the
      queue only grows: measured 1 ms -> 60 ms (t=15 s) -> 215 ms (t=29 s),
      faster with the MuJoCo viewer / Viser running.
    * joint_ctrl out: ``ctrl_pub_que`` is an unbounded FIFO emptied one command
      per publisher-thread loop, so commands can queue up as well.
    * At rest nothing moves and the delay does not matter. Once the arm is
      pushed, the spring-damper acts on q, qd from tens of ms ago. The wrist
      joints are light (joint 7: kq / M ~ (40 rad/s)^2, dq / M ~ 80 1/s), so a
      few tens of ms of delay already turns the damping into energy injection:
      a 0.4 s push at t=14 s (state 60 ms old) ended in a joint 5 speed above
      100 rad/s, while the same push at t=6 s (state 5 ms old) settled in 1.5 s.
Fix (same as ``CollisionAwareGravCompControl_v2`` / ``IKQPControl_v2``):
    drain the measurement queues (newest sample only) and keep only the newest
    unsent command.

``v2.debug_print: true`` prints once per second (sim time):
    upd      ``_update`` calls per second (1000 expected)
    age      cur_time - timestamp of the arm joint state used (ms)
    dropped  old arm samples skipped by the drain in that second
    ctrl_q   commands waiting in ctrl_pub_que (0 or 1)
"""

from __future__ import annotations

from queue import Empty

from omegaconf import DictConfig

from controller.CollisionAwareGravCompControl_v2 import _LatestOnlyQueue
from controller.ImpedanceControl import ImpedanceController
from data_type.basic_types.JointMeasData import JointMeasData


class ImpedanceControllerV2(ImpedanceController):
    def __init__(self, config: DictConfig, *args, **kwargs):
        super().__init__(config, *args, **kwargs)

        v2 = config.get("v2", {}) or {}
        self._debug_print = bool(v2.get("debug_print", False))
        self._arm_channel = str(self._dict_joints["arm"]["joint_meas_channel"])
        self._arm_meas_t = float("nan")
        self._dbg_t = -1e9
        self._dbg_upd = 0
        self._dbg_dropped = 0

    def _check_and_get_data_from_que(self) -> None:
        for joint_cfg in self._dict_joints.values():
            channel = joint_cfg["joint_meas_channel"]
            idx = joint_cfg["list_joint_idx"]
            meas = None
            n = 0
            while True:  # drain -> newest sample only
                try:
                    meas = self.intr_sub_que_dict[channel].get_nowait()
                    n += 1
                except Empty:
                    break
            if meas is None:
                continue
            if not isinstance(meas, JointMeasData):
                raise ValueError(f"Expected JointMeasData on '{channel}'")
            t_meas, q, qd, _ = meas.get_data()
            self._q[idx] = q
            self._qd[idx] = qd
            if channel == self._arm_channel:
                self._arm_meas_t = float(t_meas)
                self._dbg_dropped += n - 1

    def set_ctrl_pub_que(self, pub_que) -> None:
        """Unsent stale commands are dropped on put()."""
        super().set_ctrl_pub_que(_LatestOnlyQueue(pub_que))

    def _update(self) -> None:
        super()._update()
        if not self._debug_print:
            return
        self._dbg_upd += 1
        if self._dbg_t < -1e8:  # first call: start the 1 s window here
            self._dbg_t = self._cur_time
            self._dbg_upd = self._dbg_dropped = 0
            return
        if self._cur_time - self._dbg_t < 1.0:
            return
        print(
            f"[Impedance v2] t={self._cur_time:7.2f}  upd={self._dbg_upd:5d}/s"
            f"  age={max(0.0, 1e3 * (self._cur_time - self._arm_meas_t)):5.1f} ms"
            f"  dropped={self._dbg_dropped:4d}  ctrl_q={self.ctrl_pub_que.qsize()}"
        )
        self._dbg_t = self._cur_time
        self._dbg_upd = 0
        self._dbg_dropped = 0
