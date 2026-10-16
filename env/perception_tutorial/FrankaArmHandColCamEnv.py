"""Franka Research 3 arm + Robotiq 2F-85 env with collision info AND a fixed
RGB-D camera, for click-and-track pointing (perception + IK QP).

Combines the two existing envs without changing them:

- ``FrankaArmHandCamEnv`` (tutorial 14): joint_meas + RGB-D + camera_info.
- ``FrankaArmHandColEnv`` (tutorial 05/06): robot / static collision info,
  which the IK QP controller waits for.

Depth format. The original sim cameras publish float32 depth in meters with
``depth_factor: 1.0``, but the perception code (``convert_pixel_to_world``)
treats depth values <= 1 as invalid, which is right for a RealSense (uint16,
millimeters, ``depth_factor: 1000``) and drops everything closer than 1 m in
sim. With ``depth_factor`` > 1 in the camera config this env publishes the
RealSense format instead: uint16 ``round(depth_m * depth_factor)``, 0 =
invalid (non-finite or out of range). The same perception config then works
on the sim camera and on a RealSense.
"""

from __future__ import annotations

import numpy as np
from omegaconf import DictConfig

from data_type.basic_types.RGBDData import RGBDData
from env.calibration_tutorial.FrankaArmHandCamEnv import FrankaArmHandCamEnv
from env.franka_fr3.FrankaArmHandColEnv import FrankaArmHandColEnv
from utils.communication.lcm.image_conversion import DTYPE_TO_CHANNEL_TYPE

_UINT16_MAX = np.iinfo(np.uint16).max


class _UInt16DepthRGBDData(RGBDData):
    """RGBDData whose depth is stored like a RealSense frame: uint16, scaled by
    ``depth_factor`` (1000 = millimeters), 0 = invalid."""

    def __init__(self, height: int, width: int, depth_factor: float, name: str):
        super().__init__(
            height,
            width,
            depth_channel_type=DTYPE_TO_CHANNEL_TYPE[np.uint16],
            name=name,
        )
        self._depth_factor = float(depth_factor)

    def set_depth_image(self, depth_image: np.ndarray):
        d = np.asarray(depth_image, dtype=np.float64) * self._depth_factor
        valid = np.isfinite(d) & (d > 0.0) & (d <= _UINT16_MAX)
        self.depth_image = np.where(valid, np.round(d), 0.0).astype(np.uint16)


class FrankaArmHandColCamEnv(FrankaArmHandCamEnv, FrankaArmHandColEnv):
    """MRO: camera env -> collision env -> MujocoBaseEnv. The camera part
    publishes RGB-D / camera_info from ``_sync_extr_data_from_sim``; the
    collision part republishes col info from ``_sync_data_from_sim``."""

    def __init__(self, config: DictConfig, *args, **kwargs):
        super().__init__(config, *args, **kwargs)

    def initialize(self) -> None:
        print(
            "[Env] FrankaArmHandColCamEnv ready. Camera publishes RGB-D, "
            "env publishes collision info. Drag 'track_target' in the viewer "
            "(double-click, Ctrl + right drag)."
        )

    def _initialize_sensors(self, sensor_config) -> None:
        super()._initialize_sensors(sensor_config)
        if sensor_config is None:
            return
        for cam_name in self._list_camera_names:
            cam_cfg = sensor_config["extrinsic"][cam_name]
            depth_factor = float(cam_cfg.get("depth_factor", 1.0))
            if depth_factor > 1.0:
                self._dict_cam_data[cam_name] = _UInt16DepthRGBDData(
                    cam_cfg["height"], cam_cfg["width"], depth_factor, name=cam_name
                )
