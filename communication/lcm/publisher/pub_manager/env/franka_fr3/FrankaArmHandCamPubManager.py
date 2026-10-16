"""Env publisher: Franka Research 3 arm + Robotiq 2F-85 joint_meas and RGB-D camera (+ info)."""

from queue import Queue

from communication.lcm.publisher.BasePubManager import BaseEnvPubManager
from communication.lcm.publisher.data_publisher.CameraInfoPublisher import (
    CameraInfoPublisher,
)
from communication.lcm.publisher.data_publisher.JointMeasPublisher import (
    JointMeasPublisher,
)
from communication.lcm.publisher.data_publisher.RGBDPublisher import RGBDPublisher


class FrankaArmHandCamPubManager(BaseEnvPubManager):
    """Joints (arm+hand) on intr queues; RGBD + camera_info on extr queues."""

    def __init__(
        self,
        *args,
        arm_joint_meas_channel: str = "sw_franka_arm_joint_meas",
        hand_joint_meas_channel: str = "sw_franka_gripper_joint_meas",
        calib_camera_channel: str = "calib_camera",
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

        self.pub_que_dict["extr_pub_que_dict"] = {
            calib_camera_channel: Queue(),
            calib_camera_channel + "_info": Queue(),
        }
        self.publisher_dict[calib_camera_channel] = RGBDPublisher(
            self._lcm_instance,
            calib_camera_channel,
            self.pub_que_dict["extr_pub_que_dict"][calib_camera_channel],
        )
        self.publisher_dict[calib_camera_channel + "_info"] = CameraInfoPublisher(
            self._lcm_instance,
            calib_camera_channel + "_info",
            self.pub_que_dict["extr_pub_que_dict"][calib_camera_channel + "_info"],
        )
