from queue import Queue

from omegaconf import DictConfig

from visualizer.BaseVisManager import BaseVisManager
from visualizer.data_visualizer.FrankaPandaVisualizer import FrankaPandaVisualizer


class FrankaPandaVisManager(BaseVisManager):
    """Viser visualizer for the Franka Panda arm + Franka Hand.

    Subscribes to the arm (7) and hand (2) joint_meas_t channels the sim env
    publishes and renders the URDF in real time.
    """

    def __init__(self, config: DictConfig, *args, **kwargs):
        super().__init__(config, max_sub_freq=1000, *args, **kwargs)

    def _set_visualizers(self, config: DictConfig):
        fcfg = config["franka_panda"]
        arm_channel = fcfg["arm_joint_meas_channel"]
        hand_channel = fcfg["hand_joint_meas_channel"]

        intr = self.data_queue_dict["intr_sub_que_dict"]
        intr[arm_channel] = Queue()
        intr[hand_channel] = Queue()

        vis = FrankaPandaVisualizer(
            self.viser,
            self._lcm_instance,
            intr[arm_channel],
            intr[hand_channel],
            arm_joint_meas_channel=arm_channel,
            hand_joint_meas_channel=hand_channel,
            urdf_path=fcfg["urdf_path"],
            mesh_dir=fcfg.get("mesh_dir", ""),
            num_arm_joints=fcfg.get("num_arm_joints", 7),
            num_hand_joints=fcfg.get("num_hand_joints", 2),
            robot_col_info_channel=fcfg.get("robot_col_info_channel", ""),
            static_col_info_channel=fcfg.get("static_col_info_channel", ""),
            col_pairs=fcfg.get("col_pairs", []),
            self_col_pairs=fcfg.get("self_col_pairs", []),
            col_dist_thr=float(fcfg.get("col_dist_thr", 0.1)),
            target_pose_channel=fcfg.get("target_pose_channel", ""),
        )

        self._list_visualizers = [vis]
        print(
            f"[Visualizer] FrankaPandaVisManager initialized "
            f"(sub freq={self._max_sub_freq} Hz, vis freq={self._vis_freq} Hz)."
        )
