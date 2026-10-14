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
"""

from __future__ import annotations

from queue import Empty

from controller.CollisionAwareGravCompControl_v2 import _LatestOnlyQueue
from controller.IKQPControl import IKQPController
from data_type.basic_types.JointMeasData import JointMeasData


class IKQPControllerV2(IKQPController):
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

    def set_ctrl_pub_que(self, pub_que) -> None:
        """Unsent stale commands are dropped on put()."""
        super().set_ctrl_pub_que(_LatestOnlyQueue(pub_que))
