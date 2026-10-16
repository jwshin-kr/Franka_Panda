"""Rough camera pose -> ``tf_link_to_cam`` for the RealSense config.

The tutorial 14 calibration starts from the camera_info extrinsic, which on a
real camera is the config's ``tf_link_to_cam``. It only refines: the start has
to be roughly right (a few cm / degrees). Measure where the camera is and a
point it looks at, both in the robot base frame (x forward, y left, z up,
origin at the base), and paste the output into the config:

    python -m utils.calibration.eyeball_camera_pose --eye 0.85 0.55 0.70 --look_at 0.3 0.0 0.2

``--roll_deg`` rotates the image about the viewing direction (0 = image x axis
horizontal, image up = world up). Same OpenCV convention as the calibration
(x right, y down, z forward); ``tf_link_to_cam`` is world -> camera.
"""

from __future__ import annotations

import argparse

import numpy as np
from scipy.spatial.transform import Rotation


def cam2base_from_look_at(eye, look_at, roll_deg: float = 0.0) -> np.ndarray:
    eye = np.asarray(eye, dtype=np.float64)
    z = np.asarray(look_at, dtype=np.float64) - eye
    z /= np.linalg.norm(z)
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-6:  # looking straight up / down
        x = np.cross(z, np.array([1.0, 0.0, 0.0]))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z], axis=1) @ Rotation.from_euler("z", roll_deg, degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = eye
    return T


def tf_link_to_cam_from_cam2base(T_cam2base: np.ndarray) -> dict:
    T_w2c = np.linalg.inv(np.asarray(T_cam2base, dtype=np.float64))
    return {
        "quaternion": Rotation.from_matrix(T_w2c[:3, :3]).as_quat().tolist(),  # x, y, z, w
        "translation": T_w2c[:3, 3].tolist(),
    }


def format_tf_link_to_cam(tf: dict) -> str:
    q = ", ".join(f"{v:.6f}" for v in tf["quaternion"])
    t = ", ".join(f"{v:.6f}" for v in tf["translation"])
    return (
        "    tf_link_to_cam:\n"
        f"      quaternion: [{q}]  # [x, y, z, w]\n"
        f"      translation: [{t}]"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--eye", type=float, nargs=3, required=True, help="camera position (m, base frame)")
    parser.add_argument("--look_at", type=float, nargs=3, required=True, help="a point the camera looks at (m, base frame)")
    parser.add_argument("--roll_deg", type=float, default=0.0)
    args = parser.parse_args()
    T = cam2base_from_look_at(args.eye, args.look_at, args.roll_deg)
    print("T_cam2base (camera pose in the base frame):")
    print(np.array2string(T, precision=4, suppress_small=True))
    print("\nRealSense config (configs/sensors/realsense/*.yaml):")
    print(format_tf_link_to_cam(tf_link_to_cam_from_cam2base(T)))


if __name__ == "__main__":
    main()
