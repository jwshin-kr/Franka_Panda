"""Joint ctrl + collision-planner debug channel (v2).

Used by ``run_controller_v2.py`` for ``CollisionAwareGravCompControllerV2``: the
controller puts one ``NamedVecListData`` per plan on ``debug_channel`` (queue of
size 1, newest wins) and ``ColAwareDebugVisualizer_v2`` draws it.
"""

from queue import Queue

from communication.lcm.publisher.BasePubManager import BaseCtrlPubManager
from communication.lcm.publisher.data_publisher.JointCtrlPublisher import (
    JointCtrlPublisher,
)
from communication.lcm.publisher.data_publisher.NamedVecListPublisher import (
    NamedVecListPublisher,
)


class JointCtrlColDebugPubManagerV2(BaseCtrlPubManager):
    def __init__(
        self,
        ctrl_channel: str = "sw_franka_arm_hand_joint_ctrl",
        debug_channel: str = "sw_col_aware_debug_v2",
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.publisher_dict[ctrl_channel] = JointCtrlPublisher(
            self._lcm_instance, ctrl_channel, self.ctrl_pub_que
        )
        if debug_channel:
            self.pub_que_dict[debug_channel] = Queue(maxsize=1)
            self.publisher_dict[debug_channel] = NamedVecListPublisher(
                self._lcm_instance, debug_channel, self.pub_que_dict[debug_channel]
            )
