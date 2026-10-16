"""Tutorial 14 calibration that also saves the result for perception.

Same calibrator and config as ``run_camera_calibration.py`` (left untouched,
it only prints the result). After the optimization this writes T_cam2base
(OpenCV camera pose in the robot base frame), its inverse T_world2cam (also
as ``tf_link_to_cam`` for the RealSense config), K and the difference to the
camera_info extrinsic (sim: MuJoCo ground truth, real camera: the config's
starting guess) to ``--output``. The point tracking perception
(``tapnet_click_and_track_calib``) reads ``T_cam2base`` from that file to
express the tracked 3D point in the robot base frame.

    python run_camera_calibration_v2.py --config configs/calibration_tutorial/calib_diff_render.yaml
    python run_camera_calibration_v2.py --config configs/calibration_tutorial/calib_diff_render_hw.yaml \
        --output configs/calibration_tutorial/results/d455_1.yaml
"""

import argparse
import datetime
import os
import sys

import numpy as np
from omegaconf import OmegaConf

from utils.calibration.diff_base_to_cam import (
    CalibInterrupted,
    DiffBaseToCamCalibrator,
    _pose_error,
)
from utils.calibration.eyeball_camera_pose import (
    format_tf_link_to_cam,
    tf_link_to_cam_from_cam2base,
)
from utils.lie.se3 import invSE3

DEFAULT_OUTPUT = "configs/calibration_tutorial/results/calib_camera.yaml"


def save_result(path: str, cfg, calibrator: DiffBaseToCamCalibrator, T_cam2base):
    T_cam2base = np.asarray(T_cam2base, dtype=np.float64)
    result = {
        "camera_channel": str(cfg["rgbd_channel"]),
        "frame": "OpenCV camera (x right, y down, z forward) in the robot base frame",
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "T_cam2base": T_cam2base.tolist(),
        "T_world2cam": invSE3(T_cam2base).tolist(),
        "K": np.asarray(calibrator._K, dtype=np.float64).tolist(),
        "image_size": [int(calibrator._H), int(calibrator._W)],
    }
    # Same convention as the RealSense config: world (= base) -> camera.
    tf = tf_link_to_cam_from_cam2base(T_cam2base)
    result["tf_link_to_cam"] = tf
    if calibrator._T_gt_c2b is not None:
        # The calibrator's "Error vs GT" is relative to the camera_info
        # extrinsic: the MuJoCo ground truth in sim, the config's
        # tf_link_to_cam (starting guess) on a real camera.
        t_err, r_err = _pose_error(T_cam2base, calibrator._T_gt_c2b)
        result["diff_vs_camera_info_extrinsic"] = {"trans_mm": t_err * 1000.0, "rot_deg": r_err}
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    OmegaConf.save(OmegaConf.create(result), path)
    print(f"[Calib] Saved T_cam2base to {path}")
    print("[Calib] For the RealSense config (better start next time / original perception):")
    print(format_tf_link_to_cam(tf))


def main():
    parser = argparse.ArgumentParser(
        description="Differentiable base–camera calibration (tutorial 14), saving the result"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="configs/calibration_tutorial/calib_diff_render.yaml",
    )
    parser.add_argument("--output", type=str, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    cfg = OmegaConf.load(args.config)
    print(OmegaConf.to_yaml(cfg))
    calibrator = DiffBaseToCamCalibrator(cfg)
    try:
        T_cam2base = calibrator.run()
    except CalibInterrupted:
        print("[Calib] Stopped by user. Nothing saved.", file=sys.stderr)
        sys.exit(130)
    save_result(args.output, cfg, calibrator, T_cam2base)


if __name__ == "__main__":
    main()
