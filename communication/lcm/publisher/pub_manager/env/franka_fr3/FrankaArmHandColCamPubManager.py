"""Env publisher: Franka Research 3 arm + Robotiq 2F-85 joint_meas, collision
info, and RGB-D camera (+ info). Union of FrankaArmHandColInfoPubManager and
FrankaArmHandCamPubManager, for FrankaArmHandColCamEnv."""

from queue import Queue

from communication.lcm.publisher.data_publisher.CameraInfoPublisher import (
    CameraInfoPublisher,
)
from communication.lcm.publisher.data_publisher.RGBDPublisher import RGBDPublisher
from communication.lcm.publisher.pub_manager.env.franka_fr3.FrankaArmHandColInfoPubManager import (
    FrankaArmHandColInfoPubManager,
)


class FrankaArmHandColCamPubManager(FrankaArmHandColInfoPubManager):
    """Joints + robot col info on intr queues; static col info, RGBD and
    camera_info on extr queues."""

    def __init__(self, *args, calib_camera_channel: str = "calib_camera", **kwargs):
        super().__init__(*args, **kwargs)

        extr = self.pub_que_dict["extr_pub_que_dict"]
        extr[calib_camera_channel] = Queue()
        extr[calib_camera_channel + "_info"] = Queue()
        self.publisher_dict[calib_camera_channel] = RGBDPublisher(
            self._lcm_instance,
            calib_camera_channel,
            extr[calib_camera_channel],
        )
        self.publisher_dict[calib_camera_channel + "_info"] = CameraInfoPublisher(
            self._lcm_instance,
            calib_camera_channel + "_info",
            extr[calib_camera_channel + "_info"],
        )
