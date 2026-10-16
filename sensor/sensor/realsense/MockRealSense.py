"""Mock RealSense: a MuJoCo render of the robot cell, for testing the real-robot
camera pipeline (calibration, point tracking) without a camera.

Publishes exactly what ``RealSense`` publishes (``RGBDData``: RGB uint8, depth
uint16 = depth_m * depth_factor, aligned to colour, 0 = invalid, wall-clock
timestamp) on the same channel. The image is rendered from ``scene_xml_path``
at the robot's *measured* joints (``hw_`` joint_meas channels; with the mock
bridge those echo the commands), with the K of the sensor config. The camera
pose is the scene's ``camera_name`` camera, i.e. the "true" pose that the
calibration has to find; the sensor config's ``tf_link_to_cam`` stays what it
is on the real camera (a guess / the calibration result).

The scene's ``track_target`` mocap cube, if any, follows ``mock.target``
(static, or a circle with a lift) so the point tracking has something to track.
"""

from __future__ import annotations

import os

# Off-screen rendering in this (non-main) thread: EGL needs no window.
os.environ.setdefault("MUJOCO_GL", "egl")

import threading
import time
from queue import Empty, Queue

import cv2
import lcm
import mujoco as mj
import numpy as np

from communication.lcm.subscriber.data_subscriber.JointMeasSubscriber import (
    JointMeasSubscriber,
)
from data_type.basic_types.JointMeasData import JointMeasData
from data_type.basic_types.RGBDData import RGBDData
from sensor.BaseSensor import BaseSensor
from utils.gripper import robotiq_2f85 as rq

_ARM_JOINTS = [f"joint{i + 1}" for i in range(7)]


class MockRealSense(BaseSensor):
    def __init__(
        self,
        name: str,
        *args,
        height: int = 480,
        width: int = 640,
        frequency: int = 30,
        fx: float = 0.0,
        fy: float = 0.0,
        cx: float = 0.0,
        cy: float = 0.0,
        depth_factor: float = 1000.0,
        mock_cfg=None,
        visualize: bool = False,
        **kwargs,
    ):
        super().__init__(name, *args, sensor_info_flag=True, **kwargs)
        mock_cfg = mock_cfg or {}
        self._width = int(width)
        self._height = int(height)
        self._dt = 1.0 / float(frequency)
        self._K = (float(fx), float(fy), float(cx), float(cy))
        self._depth_factor = float(depth_factor)
        self._scene = str(mock_cfg.get("scene_xml_path", "assets/scene/franka_fr3/fr3_cam_track.xml"))
        self._camera_name = str(mock_cfg.get("camera_name", "calib_camera"))
        self._arm_channel = str(mock_cfg.get("arm_joint_meas_channel", "hw_franka_arm_joint_meas"))
        self._hand_channel = str(mock_cfg.get("hand_joint_meas_channel", "hw_franka_gripper_joint_meas"))
        self._q = np.asarray(
            mock_cfg.get("default_q", [0.0, 0.0, 0.0, -1.5707963, 0.0, 1.5707963, 0.0, 0.0]),
            dtype=np.float64,
        )
        self._target_cfg = mock_cfg.get("target", {}) or {}
        self._visualize = visualize
        self._latest_vis = None
        self._window_name = f"MockRealSense [{name}] (MuJoCo render)"

        self._lcm = lcm.LCM()
        self._arm_q: Queue = Queue()
        self._hand_q: Queue = Queue()
        JointMeasSubscriber(self._lcm, self._arm_q, num_joints=7).subscribe(self._arm_channel)
        JointMeasSubscriber(self._lcm, self._hand_q, num_joints=1).subscribe(self._hand_channel)
        self._lcm_thread = threading.Thread(target=self._lcm_loop, daemon=True)

    # ------------------------------------------------------------------
    def _lcm_loop(self):
        while not self._stop_event.is_set():
            self._lcm.handle_timeout(10)

    def _latest_joints(self):
        for que, sl in ((self._arm_q, slice(0, 7)), (self._hand_q, slice(7, 8))):
            meas = None
            while True:
                try:
                    meas = que.get_nowait()
                except Empty:
                    break
            if isinstance(meas, JointMeasData):
                self._q[sl] = meas.get_q()
        return self._q.copy()

    def _target_pos(self, t: float, p0: np.ndarray) -> np.ndarray:
        cfg = self._target_cfg
        if cfg.get("motion", "static") != "circle":
            return p0
        s = t - float(cfg.get("start_delay", 10.0))
        if s <= 0.0:
            return p0
        period = float(cfg.get("period", 15.0))
        cycles = cfg.get("cycles", None)
        if cycles is not None:
            s = min(s, float(cycles) * period)  # stops back at the start
        w = 2.0 * np.pi / period
        r = float(cfg.get("radius", 0.10))
        z = float(cfg.get("lift", 0.08))
        return p0 + np.array(
            [r * np.sin(w * s), r * (1.0 - np.cos(w * s)), z * (1.0 - np.cos(w * s)) / 2.0]
        )

    def _build_model(self):
        spec = mj.MjSpec.from_file(self._scene)
        cam = spec.camera(self._camera_name)
        fx, fy, cx, cy = self._K
        W, H = self._width, self._height
        # Pixel intrinsics (MuJoCo's x principal offset is mirrored w.r.t. OpenCV).
        cam.sensor_size = [W, H]
        cam.resolution = [W, H]
        cam.focal_pixel = [fx, fy]
        cam.principal_pixel = [W / 2.0 - cx, cy - H / 2.0]
        model = spec.compile()
        return model, mj.MjData(model)

    def _run(self):
        model, data = self._build_model()
        renderer = mj.Renderer(model, self._height, self._width)
        arm_adr = [model.joint(n).qposadr[0] for n in _ARM_JOINTS]
        link_adr = {n: model.joint(n).qposadr[0] for n in rq.MJ_LINKAGE}
        has_target = model.nmocap > 0 and mj.mj_name2id(
            model, mj.mjtObj.mjOBJ_BODY, "track_target"
        ) >= 0
        p0 = data.mocap_pos[0].copy() if has_target else None
        self._lcm_thread.start()
        print(
            f"[MockRealSense {self.name}] MOCK camera: MuJoCo render of '{self._scene}' "
            f"camera '{self._camera_name}' at the measured joints of "
            f"'{self._arm_channel}' / '{self._hand_channel}'."
        )

        t0 = time.time()
        next_t = t0
        while not self._stop_event.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(min(0.002, next_t - now))
                continue
            next_t += self._dt
            if next_t < now:
                next_t = now + self._dt

            q = self._latest_joints()
            data.qpos[arm_adr] = q[:7]
            for n, k in rq.MJ_LINKAGE.items():
                data.qpos[link_adr[n]] = k * q[7]
            if has_target:
                data.mocap_pos[0] = self._target_pos(now - t0, p0)
            mj.mj_forward(model, data)

            renderer.update_scene(data, camera=self._camera_name)
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=self._camera_name)
            depth_m = renderer.render().copy()
            renderer.disable_depth_rendering()
            d = depth_m.astype(np.float64) * self._depth_factor
            valid = np.isfinite(d) & (d > 0.0) & (d <= np.iinfo(np.uint16).max)
            depth = np.where(valid, np.round(d), 0.0).astype(np.uint16)

            out = RGBDData(
                height=self._height,
                width=self._width,
                rgb_channel_type=1,
                depth_channel_type=3,
                name=self.name,
            )
            out.set_data(
                t=self.get_time(),
                rgb_channel_type=1,
                rgb_image=rgb,
                depth_channel_type=3,
                depth_image=depth,
            )
            self.pub_data_queue.put(out)

            if self._visualize:
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET
                )
                self._latest_vis = np.hstack((cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), depth_vis))
        renderer.close()

    def get_latest_vis(self):
        """(window_name, image) for the main-thread display pump, or None."""
        img = self._latest_vis
        if img is None:
            return None
        return self._window_name, img
