"""TapnetClickAndTrack with the camera pose taken from the calibration result.

The original node (``TapnetClickAndTrack``, unchanged) converts the tracked
pixel + depth to 3D with ``cam2world = inv(CameraInfo.extrinsic)``. In sim that
extrinsic is MuJoCo's ground truth, on a real camera it is whatever the camera
driver publishes. Here ``cam2world`` is replaced by ``T_cam2base`` from the
tutorial 14 calibration (``run_camera_calibration_v2.py``), so the published
point is in the robot base frame as estimated by the calibration. Everything
else (click, TAPIR tracking, depth lookup, publishing) is the original code.

``cam2base_path: null`` keeps the original behaviour (CameraInfo extrinsic).
"""

from __future__ import annotations

import numpy as np
from omegaconf import OmegaConf

from perception.tapnet.TapnetClickAndTrack import TapnetClickAndTrack
from utils.lie.so3 import log_SO3, unskew


class TapnetClickAndTrackCalib(TapnetClickAndTrack):
    def __init__(self, config):
        super().__init__(config)
        self.name = "TapnetClickAndTrackCalib"
        self._cam2base_path = config.get("cam2base_path", None)
        self._T_cam2base = None
        if self._cam2base_path:
            calib = OmegaConf.load(self._cam2base_path)
            T = np.asarray(calib["T_cam2base"], dtype=np.float64)
            if T.shape != (4, 4):
                raise ValueError(
                    f"T_cam2base in {self._cam2base_path} must be 4x4, got {T.shape}"
                )
            self._T_cam2base = T

    def initialize(self):
        # GUI + wait for CameraInfo (K, depth factor, image size, extrinsic).
        super().initialize()
        if self._T_cam2base is None:
            print("[Perception] cam2base_path not set: using the CameraInfo extrinsic.")
            return
        T_info = self._cam2world.copy()
        self._cam2world = self._T_cam2base.copy()
        dT = self._cam2world @ np.linalg.inv(T_info)
        t_mm = 1000.0 * float(np.linalg.norm(dT[:3, 3]))
        r_deg = float(np.rad2deg(np.linalg.norm(unskew(log_SO3(dT[:3, :3])))))
        print(f"[Perception] Using T_cam2base from {self._cam2base_path}:\n{self._cam2world}")
        print(
            f"[Perception] Calibration vs CameraInfo extrinsic "
            f"(sim: ground truth, RealSense: config tf_link_to_cam): "
            f"{t_mm:.1f} mm, {r_deg:.2f} deg"
        )
