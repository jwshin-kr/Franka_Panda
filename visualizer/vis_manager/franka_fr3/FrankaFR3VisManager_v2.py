"""FrankaFR3VisManager + the collision-aware debug overlay (v2)."""

from omegaconf import DictConfig

from visualizer.data_visualizer.ColAwareDebugVisualizer_v2 import (
    ColAwareDebugVisualizerV2,
)
from visualizer.vis_manager.franka_fr3.FrankaFR3VisManager import (
    FrankaFR3VisManager,
)


class FrankaFR3VisManagerV2(FrankaFR3VisManager):
    """Original robot / collision-geometry visualizer, plus
    ``ColAwareDebugVisualizerV2`` configured by the ``col_aware_debug_v2``
    section (see configs/visualizer/franka_fr3/collision_v2.yaml)."""

    def _set_visualizers(self, config: DictConfig):
        super()._set_visualizers(config)
        fcfg = config["franka_fr3"]
        dcfg = dict(config.get("col_aware_debug_v2", {}) or {})
        dbg = ColAwareDebugVisualizerV2(
            self.viser,
            self._lcm_instance,
            urdf_path=fcfg["urdf_path"],
            mesh_dir=fcfg.get("mesh_dir", ""),
            arm_joint_meas_channel=fcfg["arm_joint_meas_channel"],
            num_arm_joints=fcfg.get("num_arm_joints", 7),
            num_joints=fcfg.get("num_arm_joints", 7) + fcfg.get("num_hand_joints", 1),
            **dcfg,
        )
        self._list_visualizers.append(dbg)
        print("[Visualizer] ColAwareDebugVisualizerV2 added.")
