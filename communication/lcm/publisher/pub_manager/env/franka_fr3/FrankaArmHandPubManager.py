from queue import Queue

from communication.lcm.publisher.data_publisher.JointMeasPublisher import (
    JointMeasPublisher,
)
from communication.lcm.publisher.BasePubManager import BaseEnvPubManager


class FrankaArmHandPubManager(BaseEnvPubManager):
    """Env-side publisher for the Franka Research 3 arm + Robotiq 2F-85.

    Publishes two joint_meas_t streams: the 7-DoF arm and the 1-DoF gripper
    (robotiq_right_driver_joint), on separate channels. The channel names
    must match the meas channels the SimFrankaFR3 platform publishes to.
    """

    def __init__(
        self,
        *args,
        arm_joint_meas_channel: str = "sw_franka_arm_joint_meas",
        hand_joint_meas_channel: str = "sw_franka_gripper_joint_meas",
        **kwargs,
    ):
        super().__init__(*args, **kwargs)

        self.pub_que_dict["intr_pub_que_dict"] = {
            arm_joint_meas_channel: Queue(),
            hand_joint_meas_channel: Queue(),
        }

        self.publisher_dict[arm_joint_meas_channel] = JointMeasPublisher(
            self._lcm_instance,
            arm_joint_meas_channel,
            self.pub_que_dict["intr_pub_que_dict"][arm_joint_meas_channel],
        )
        self.publisher_dict[hand_joint_meas_channel] = JointMeasPublisher(
            self._lcm_instance,
            hand_joint_meas_channel,
            self.pub_que_dict["intr_pub_que_dict"][hand_joint_meas_channel],
        )
