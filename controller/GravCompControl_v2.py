"""Gravity compensation, v2: tutorial 05 cause 1 (stale LCM data) fixed, for the
real robot. The original controller (``GravCompControl.py``) is left untouched.

The original pops one joint_meas per channel per loop iteration from an
unbounded FIFO and sends commands through another FIFO, so any backlog makes
it compensate gravity for an older pose than the arm is in. Same fix as
``IKQPControl_v2``: drain the measurement queues (newest sample only) and keep
only the newest unsent command.
"""

from __future__ import annotations

from queue import Empty

from controller.CollisionAwareGravCompControl_v2 import _LatestOnlyQueue
from controller.GravCompControl import GravCompController
from data_type.basic_types.JointMeasData import JointMeasData


class GravCompControllerV2(GravCompController):
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
            self._q[idx] = q
            self._qd[idx] = qd

    def set_ctrl_pub_que(self, pub_que) -> None:
        """Unsent stale commands are dropped on put()."""
        super().set_ctrl_pub_que(_LatestOnlyQueue(pub_que))
