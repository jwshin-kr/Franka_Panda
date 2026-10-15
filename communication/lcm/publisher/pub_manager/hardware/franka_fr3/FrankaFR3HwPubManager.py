from queue import Queue

from communication.lcm.publisher.data_publisher.ColInfoPublisher import (
    ColInfoPublisher,
)
from communication.lcm.publisher.data_publisher.JointMeasPublisher import (
    JointMeasPublisher,
)
from communication.lcm.publisher.BasePubManager import BaseEnvPubManager


class FrankaFR3HwPubManager(BaseEnvPubManager):
    """Hardware-side publisher for the Franka Research 3 + Robotiq 2F-85.

    The ARM measurement is published by the C++ bridge on its own LCM
    instance, so the Python side publishes:

    - the gripper joint_meas (robotiq_right_driver_joint, 1 joint), on the
      hw_ twin of the sim's channel (sw_franka_gripper_joint_meas),
    - optionally (when the channel names are given) the robot / static
      collision info, like the franka_fr3_col env does in the sim, so the
      collision-aware controllers (tutorials 05 / 06) run unchanged on the
      real robot.
    """

    def __init__(
        self,
        *args,
        hand_joint_meas_channel: str = "hw_franka_gripper_joint_meas",
        robot_col_info_channel: str = "",
        static_col_info_channel: str = "",
        **kwargs,
    ):
        super().__init__(*args, **kwargs)

        intr = {hand_joint_meas_channel: Queue()}
        extr = {}
        self.publisher_dict[hand_joint_meas_channel] = JointMeasPublisher(
            self._lcm_instance, hand_joint_meas_channel, intr[hand_joint_meas_channel]
        )
        if robot_col_info_channel:
            intr[robot_col_info_channel] = Queue()
            self.publisher_dict[robot_col_info_channel] = ColInfoPublisher(
                self._lcm_instance, robot_col_info_channel, intr[robot_col_info_channel]
            )
        if static_col_info_channel:
            extr[static_col_info_channel] = Queue()
            self.publisher_dict[static_col_info_channel] = ColInfoPublisher(
                self._lcm_instance, static_col_info_channel, extr[static_col_info_channel]
            )
        self.pub_que_dict["intr_pub_que_dict"] = intr
        self.pub_que_dict["extr_pub_que_dict"] = extr
