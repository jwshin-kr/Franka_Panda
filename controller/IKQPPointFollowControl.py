"""IK QP (v2) whose 3D target follows a point published by perception.

The tracked point (``point_set_t`` on ``sub_manager.point_channel``, robot base
frame, e.g. from ``TapnetClickAndTrackCalib``) replaces the keyboard target of
``IKQPControllerV2``. Everything downstream is the tutorial 06 v2 controller
unchanged: contracting field -> IK QP with collision constraints -> joint
reference tracking.

Target = point + ``point_follow.offset`` (default 10 cm above the point, so the
gripper points at it without touching it), then

* clamped to a workspace box and a reach sphere around the shoulder, so a bad
  depth / tracking sample cannot send the arm somewhere unreachable or into
  the cell;
* low-pass filtered (time constant ``filter_tau``) against pixel / depth
  jitter. Speed is limited downstream by the controller's ``ref_v_max``;
* optionally (``max_jump``) a sudden jump is ignored until it persists.

When no point arrives for ``timeout`` s (tracking lost, invalid depth,
perception stopped) the last target is held. Keys: ``t`` toggles following
(off: the IK QP keyboard nudging works as in tutorial 06), all IK QP keys
(``h`` home, ``o``/``c`` gripper, ``p``, ``q``) are unchanged; while following,
the next point overrides a keyboard nudge.
"""

from __future__ import annotations

import time
from queue import Empty

import numpy as np

from controller.IKQPControl_v2 import IKQPControllerV2
from data_type.basic_types.Point3DData import Point3DData


class IKQPPointFollowController(IKQPControllerV2):
    def __init__(self, config, *args, **kwargs):
        super().__init__(config, *args, **kwargs)
        self._point_channel = config["sub_manager"]["point_channel"]

        pf = config.get("point_follow", {}) or {}
        self._following = bool(pf.get("enabled", True))
        self._pf_offset = np.asarray(pf.get("offset", [0.0, 0.0, 0.10]), dtype=np.float64)
        self._pf_min = np.asarray(pf.get("workspace_min", [0.1, -0.3, 0.05]), dtype=np.float64)
        self._pf_max = np.asarray(pf.get("workspace_max", [0.8, 0.6, 0.9]), dtype=np.float64)
        self._pf_reach_center = np.asarray(
            pf.get("reach_center", [0.0, 0.0, 0.333]), dtype=np.float64
        )
        self._pf_reach_radius = float(pf.get("reach_radius", 0.75))
        self._pf_filter_tau = float(pf.get("filter_tau", 0.1))
        self._pf_timeout = float(pf.get("timeout", 0.5))
        self._pf_print_dt = float(pf.get("print_dt", 1.0))
        # Jump rejection (off with max_jump null): a new target farther than
        # max_jump from the current one is ignored until it has persisted
        # jump_accept_after s (tracking slipping onto the background, a bad
        # depth pixel).
        max_jump = pf.get("max_jump", None)
        self._pf_max_jump = None if max_jump is None else float(max_jump)
        self._pf_jump_accept = float(pf.get("jump_accept_after", 0.5))
        self._pf_jump_since = None

        self._pf_target = None  # filtered target (base frame)
        self._pf_last_rx = None  # wall time of the last point
        self._pf_lost = False
        self._pf_last_print = 0.0
        print(
            f"[PointFollow] point channel '{self._point_channel}', offset {self._pf_offset}, "
            f"following {'on' if self._following else 'off'} (key 't' toggles)"
        )

    # ------------------------------------------------------------ keyboard
    def _handle_key(self, key: str) -> None:
        if key == "t":
            self._following = not self._following
            if self._following:
                self._pf_target = None  # restart the filter at the next point
                self._enter_ik_mode("point follow")
            print(f"[PointFollow] following {'on' if self._following else 'off'}")
            return
        super()._handle_key(key)

    # -------------------------------------------------------- measurements
    def _check_and_get_data_from_que(self) -> None:
        super()._check_and_get_data_from_que()
        point = None
        while True:  # newest point only
            try:
                point = self.extr_sub_que_dict[self._point_channel].get_nowait()
            except Empty:
                break
        now = time.time()
        if point is None:
            if (
                self._following
                and self._pf_last_rx is not None
                and not self._pf_lost
                and now - self._pf_last_rx > self._pf_timeout
            ):
                self._pf_lost = True
                print(f"[PointFollow] no point for {self._pf_timeout:.1f} s: holding the target")
            return
        if not isinstance(point, Point3DData) or point.get_num_points() < 1:
            return
        p = point.get_position()[0]
        if not np.all(np.isfinite(p)):
            return
        self._on_point(p, now)

    def _on_point(self, p: np.ndarray, now: float) -> None:
        dt = None if self._pf_last_rx is None else now - self._pf_last_rx
        self._pf_last_rx = now
        if self._pf_lost:
            self._pf_lost = False
            print("[PointFollow] point received again")
        if not self._following:
            return

        raw = self._clamp_to_workspace(p + self._pf_offset)
        if (
            self._pf_max_jump is not None
            and self._pf_target is not None
            and float(np.linalg.norm(raw - self._pf_target)) > self._pf_max_jump
        ):
            if self._pf_jump_since is None:
                self._pf_jump_since = now
                print(
                    f"[PointFollow] target jump > {self._pf_max_jump:.2f} m ignored "
                    f"(accepted if it persists {self._pf_jump_accept:.1f} s)"
                )
            if now - self._pf_jump_since < self._pf_jump_accept:
                return
            print("[PointFollow] jump persisted: following the new point")
        self._pf_jump_since = None
        if self._pf_target is None or dt is None or dt > self._pf_timeout:
            self._pf_target = raw.copy()
        else:
            a = 1.0 - np.exp(-dt / max(self._pf_filter_tau, 1e-6))
            self._pf_target = self._pf_target + a * (raw - self._pf_target)

        with self._meas_lock:
            if self._ctrl_mode != "ik":
                return  # e.g. homing ('h'); 't' twice re-enters ik mode
            self._p_des = self._pf_target.copy()
            self._target_initialized = True

        if now - self._pf_last_print > self._pf_print_dt:
            self._pf_last_print = now
            t = self._pf_target
            print(
                f"[PointFollow] point [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] -> "
                f"target [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}], "
                f"tip [{self._tip[0]:.3f}, {self._tip[1]:.3f}, {self._tip[2]:.3f}]"
            )

    def _clamp_to_workspace(self, p: np.ndarray) -> np.ndarray:
        p = np.clip(p, self._pf_min, self._pf_max)
        r = p - self._pf_reach_center
        n = float(np.linalg.norm(r))
        if n > self._pf_reach_radius:
            p = self._pf_reach_center + r * (self._pf_reach_radius / n)
        return p
