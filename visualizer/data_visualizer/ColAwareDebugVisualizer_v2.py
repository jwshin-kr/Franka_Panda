"""Viser overlay that makes the three causes of the "jumpy robot" visible (v2).

Works with BOTH controllers:
    * original ``run_controller.py`` -> cause 1 (ghost + lag) and cause 3
      (qd_des / gain plots) from the joint_ctrl channel alone;
    * ``run_controller_v2.py`` -> additionally cause 2 (witness points, trails,
      jump counter) and v* arrows from the planner debug channel. Run it with
      ``v2.fix_*=false`` to see the original behaviour, then with the defaults.

What to look at
    0) Collision spheres: the "Show collision spheres (v2)" checkbox (next to the
       original "Show collision geometry" checkbox, which shows the cylinders)
       draws the sphere chain the v2 planner measures distances with. It is
       built from the same robot_col_info primitives with the same
       spheres_from_primitive() as the controller, placed with the measured q.
    1) Stale data: the red translucent "ghost" robot is the joint state the
       controller used for the command that is being applied right now
       (q_des - qd_des*dt of the newest joint_ctrl). It should sit on top of
       the real robot; if it trails behind, the robot is driven by old data.
       The panel splits the delay into the joint_meas FIFO (into the
       controller) and the joint_ctrl FIFO (out of the controller).
    2) Witness jumps: small spheres = nearest (witness) points the planner
       used; the thin yellow trail is their recent path. A RED trail segment is
       a jump > witness_jump_thr between two plans.
    3) Command: cyan arrows = desired escape velocity v* at each witness point
       (length = v* x arrow scale). Plots: wrist qd_des (bang-bang between the
       +/- limits = saturated, flipping command) and the gain activation
       (0 = free-float, 1 = full kp/kd; square wave = hard switching).
"""

from __future__ import annotations

import queue
import time
from collections import deque

import numpy as np
import trimesh
import viser.uplot as uplot
from viser.extras import ViserUrdf
from yourdfpy import URDF

from communication.lcm.subscriber.data_subscriber.ColInfoSubscriber import (
    ColInfoSubscriber,
)
from communication.lcm.subscriber.data_subscriber.JointCtrlSubsriber import (
    JointCtrlSubscriber,
)
from communication.lcm.subscriber.data_subscriber.JointMeasSubscriber import (
    JointMeasSubscriber,
)
from communication.lcm.subscriber.data_subscriber.NamedVecListSubscriber import (
    NamedVecListSubscriber,
)
from data_type.basic_types.ColInfoData import ColInfoData
from data_type.basic_types.JointCtrlData import JointCtrlData
from data_type.basic_types.JointMeasData import JointMeasData
from data_type.basic_types.NamedVecListData import NamedVecListData
from utils.collision.sphere_swept_v2 import spheres_from_primitive

STATE_NAME = {0: "grav_comp", 1: "repulsive", 2: "collision"}

_TRAIL_COLOR = np.array([255, 220, 60], dtype=np.uint8)  # yellow
_JUMP_COLOR = np.array([255, 30, 30], dtype=np.uint8)  # red
_VSTAR_COLOR = np.array([60, 200, 255], dtype=np.uint8)  # cyan
_EMPTY_SEG = np.zeros((1, 2, 3), dtype=np.float32)
_SPHERE_COLOR = [165, 105, 245, 100]  # translucent purple: v2 collision spheres
# Links the v2 controller covers with spheres (col_link_names in its YAML).
_SPHERE_LINKS = ("link1", "link2", "link3", "link4", "link5", "link6", "link7", "hand")


def _act_color(act: np.ndarray) -> np.ndarray:
    """0 -> green, 0.5 -> yellow, 1 -> red."""
    a = np.clip(np.asarray(act, dtype=float), 0.0, 1.0)[:, None]
    green = np.array([40, 200, 80], dtype=float)
    yellow = np.array([255, 210, 40], dtype=float)
    red = np.array([255, 40, 40], dtype=float)
    lo = green + (yellow - green) * np.clip(2 * a, 0, 1)
    return np.where(a < 0.5, lo, yellow + (red - yellow) * np.clip(2 * a - 1, 0, 1)).astype(np.uint8)


class ColAwareDebugVisualizerV2:
    def __init__(
        self,
        viser_server,
        lcm_instance,
        urdf_path: str,
        mesh_dir: str = "",
        arm_joint_meas_channel: str = "sw_franka_arm_joint_meas",
        ctrl_channel: str = "sw_franka_arm_hand_joint_ctrl",
        debug_channel: str = "sw_col_aware_debug_v2",
        num_arm_joints: int = 7,
        num_joints: int = 9,
        ctrl_dt: float = 0.001,
        kp_multiplier: float = 1.0,
        kd_nominal_joint1: float = 20.0,
        qd_limit: float = 2.0,
        plot_joints=(5, 6, 7),
        plot_history_s: float = 6.0,
        lag_buffer_s: float = 5.0,
        trail_s: float = 0.6,
        witness_jump_thr: float = 0.03,
        vstar_arrow_scale: float = 0.1,
        robot_col_info_channel: str = "sw_franka_arm_hand_robot_col_info",
        sphere_links=_SPHERE_LINKS,
        sphere_spacing_ratio: float = 1.0,
        **_unused,
    ):
        self._server = viser_server
        self._n_arm = int(num_arm_joints)
        self._n = int(num_joints)
        self._ctrl_dt = float(ctrl_dt)
        self._kp_mult = float(kp_multiplier)
        self._kd_nom = float(kd_nominal_joint1)
        self._qd_limit = float(qd_limit)
        self._plot_joints = [int(j) - 1 for j in plot_joints]
        self._trail_s = float(trail_s)
        self._jump_thr = float(witness_jump_thr)
        self._arrow_scale = float(vstar_arrow_scale)

        # ---- LCM (a second subscription on already-used channels is fine)
        self._meas_q: queue.Queue = queue.Queue()
        self._ctrl_q: queue.Queue = queue.Queue()
        self._dbg_q: queue.Queue = queue.Queue()
        JointMeasSubscriber(lcm_instance, self._meas_q, self._n_arm).subscribe(arm_joint_meas_channel)
        JointCtrlSubscriber(lcm_instance, self._ctrl_q, self._n).subscribe(ctrl_channel)
        if debug_channel:
            NamedVecListSubscriber(lcm_instance, self._dbg_q).subscribe(debug_channel)
        self._col_q: queue.Queue = queue.Queue()
        if robot_col_info_channel:
            ColInfoSubscriber(lcm_instance, self._col_q).subscribe(robot_col_info_channel)

        # ---- 0) collision spheres (same model as the v2 planner)
        self._sphere_links = [str(n) for n in sphere_links]
        self._sphere_spacing = float(sphere_spacing_ratio)
        self._sphere_nodes: list = []  # (handle, link name, centre in the link frame)
        self._sphere_sig = None
        self._q_arm_latest: np.ndarray | None = None

        # ---- cause 1 buffers: measured arm q history, latest command
        cap = int(lag_buffer_s * 1000) + 10
        self._buf_t = np.full(cap, np.nan)
        self._buf_q = np.full((cap, self._n_arm), np.nan)
        self._buf_i = 0
        self._last_ctrl: JointCtrlData | None = None
        self._lag_ms: float | None = None
        self._lag_in_ms: float | None = None
        self._lag_out_ms: float | None = None
        self._lag_still = False
        self._lag_hist = deque(maxlen=90)

        # ---- cause 2/3 state from the debug channel
        self._trails: dict[str, deque] = {}
        self._last_dbg: dict | None = None
        self._last_dbg_wall = 0.0

        # ---- plots
        n_plot = max(64, int(plot_history_s * 100))
        self._pt = deque(maxlen=n_plot)
        self._pq = deque(maxlen=n_plot)
        self._pa = deque(maxlen=n_plot)
        self._last_plot_t = -1e9

        self._build_scene(urdf_path, mesh_dir)
        self._build_gui()

    # ------------------------------------------------------------------ build
    def _build_scene(self, urdf_path, mesh_dir):
        kwargs = dict(load_meshes=True, build_scene_graph=True,
                      load_collision_meshes=False, build_collision_scene_graph=False)
        if mesh_dir:
            kwargs["mesh_dir"] = mesh_dir
        self._ghost = ViserUrdf(
            self._server,
            urdf_or_path=URDF.load(urdf_path, **kwargs),
            root_node_name="/v2dbg/ghost_robot",
            load_meshes=True,
            load_collision_meshes=False,
            mesh_color_override=(1.0, 0.15, 0.15, 0.35),
        )
        # plain FK model for placing the collision spheres (no meshes needed)
        fk_kwargs = dict(load_meshes=False, build_scene_graph=True,
                         load_collision_meshes=False, build_collision_scene_graph=False)
        if mesh_dir:
            fk_kwargs["mesh_dir"] = mesh_dir
        self._fk = URDF.load(urdf_path, **fk_kwargs)
        # ghost = moving chain only (link1..link7 + hand); hide base/enclosure
        moving = {f"link{i}" for i in range(1, 8)} | {"hand"}
        for mesh in self._ghost._meshes:
            if not any(part in moving for part in str(mesh.name).split("/")):
                mesh.visible = False
        self._ghost.show_visual = False
        self._ghost_on = False
        sc = self._server.scene
        self._pts = sc.add_point_cloud("/v2dbg/witness", points=np.zeros((1, 3), np.float32),
                                       colors=np.zeros((1, 3), np.uint8), point_size=0.018,
                                       point_shape="circle", visible=False)
        self._obs_lines = sc.add_line_segments("/v2dbg/witness_to_obstacle", points=_EMPTY_SEG,
                                               colors=(200, 200, 200), thickness=1.5,
                                               thickness_units="screen", visible=False)
        self._trail_lines = sc.add_line_segments("/v2dbg/witness_trails", points=_EMPTY_SEG,
                                                 colors=(255, 220, 60), thickness=3.0,
                                                 thickness_units="screen", visible=False)
        self._arrows = sc.add_arrows("/v2dbg/vstar", points=_EMPTY_SEG, colors=_VSTAR_COLOR,
                                     shaft_radius=0.005, head_radius=0.011, head_length=0.025,
                                     visible=False)

    def _build_gui(self):
        gui = self._server.gui
        # Same style and level as the original "Show collision geometry" toggle
        # (cylinders + walls), so the two models can be switched independently.
        self._cb_spheres = gui.add_checkbox(
            "Show collision spheres (v2)", initial_value=False,
            hint="Sphere chain the v2 planner uses instead of one cylinder per link.",
        )

        @self._cb_spheres.on_update
        def _(_):
            on = bool(self._cb_spheres.value)
            for handle, _ln, _c in self._sphere_nodes:
                handle.visible = on
            if on:
                self._update_spheres()

        with gui.add_folder("Col-aware debug (v2)"):
            self._md = gui.add_markdown("_waiting for joint_ctrl ..._")
            self._cb_ghost = gui.add_checkbox("1) ghost = controller's joint state", True)
            self._cb_witness = gui.add_checkbox("2) witness points + trails", True)
            self._cb_vstar = gui.add_checkbox("3) v* arrows", True)
            t0 = np.zeros(2)
            self._plot_qd = gui.add_uplot(
                data=(t0, t0, t0, t0, t0, t0),
                series=(
                    uplot.Series(label="t"),
                    uplot.Series(label=f"j{self._plot_joints[0] + 1}", stroke="#d33682", width=2),
                    uplot.Series(label=f"j{self._plot_joints[1] + 1}", stroke="#268bd2", width=2),
                    uplot.Series(label=f"j{self._plot_joints[2] + 1}", stroke="#859900", width=2),
                    uplot.Series(label="+lim", stroke="#999999", width=1),
                    uplot.Series(label="-lim", stroke="#999999", width=1),
                ),
                title="3) wrist qd_des [rad/s]",
                aspect=2.0,
                height=200,
            )
            self._plot_alpha = gui.add_uplot(
                data=(t0, t0),
                series=(uplot.Series(label="t"),
                        uplot.Series(label="gain", stroke="#cb4b16", width=2)),
                title="3) gain activation kd/kd_nom (0=free, 1=full)",
                aspect=3.0,
                height=140,
            )

        @self._cb_ghost.on_update
        def _(_):
            self._set_ghost_visible(bool(self._cb_ghost.value) and self._last_ctrl is not None)

    def _set_ghost_visible(self, on: bool) -> None:
        if on != self._ghost_on:
            self._ghost.show_visual = on
            self._ghost_on = on

    # ------------------------------------------------------------------ polls
    def _poll_meas(self):
        cap = len(self._buf_t)
        while True:
            try:
                m = self._meas_q.get_nowait()
            except queue.Empty:
                break
            if not isinstance(m, JointMeasData):
                continue
            t, q, _, _ = m.get_data()
            i = self._buf_i % cap
            self._buf_t[i] = t
            self._buf_q[i] = q[: self._n_arm]
            self._buf_i += 1
            self._q_arm_latest = np.asarray(q[: self._n_arm], dtype=float)

    def _poll_ctrl(self):
        while True:
            try:
                c = self._ctrl_q.get_nowait()
            except queue.Empty:
                break
            if not isinstance(c, JointCtrlData):
                continue
            self._last_ctrl = c
            t, _, qd_des, _, _, kd = c.get_data()
            if t - self._last_plot_t >= 0.01:  # 100 Hz plot samples
                self._last_plot_t = t
                self._pt.append(t)
                self._pq.append(qd_des[self._plot_joints])
                self._pa.append(kd[0] / self._kd_nom if self._kd_nom > 0 else 0.0)

    def _poll_debug(self):
        now_t = None
        while True:
            try:
                d = self._dbg_q.get_nowait()
            except queue.Empty:
                break
            if not isinstance(d, NamedVecListData):
                continue
            t, names, vecs = d.get_data()
            parsed = {"t": t, "rows": {}}
            for name, v in zip(names, vecs):
                if "|" not in name:
                    parsed[name] = np.asarray(v, dtype=float)
                    continue
                kind, key = name.split("|", 1)
                parsed["rows"].setdefault(key, {})[kind] = np.asarray(v, dtype=float)
            # trails from EVERY plan (not only the last one of this frame)
            for key, r in parsed["rows"].items():
                if "p" in r:
                    self._trails.setdefault(key, deque(maxlen=200)).append((t, r["p"].copy()))
            self._last_dbg = parsed
            self._last_dbg_wall = time.time()
            now_t = t
        if now_t is not None:
            for key in list(self._trails):
                tr = self._trails[key]
                while tr and now_t - tr[0][0] > self._trail_s:
                    tr.popleft()
                if not tr:
                    del self._trails[key]

    # ---------------------------------------------------------------- 0) spheres
    def _poll_col_info(self):
        data = None
        while True:
            try:
                data = self._col_q.get_nowait()
            except queue.Empty:
                break
        if not isinstance(data, ColInfoData):
            return
        _, geoms = data.get_data()
        sig = tuple(
            (name, geoms[name]["type"], tuple(np.round(np.asarray(geoms[name]["size"], dtype=float), 5)))
            for name in self._sphere_links if name in geoms
        )
        if sig == self._sphere_sig:
            return
        self._sphere_sig = sig
        for handle, _ln, _c in self._sphere_nodes:
            handle.remove()
        self._sphere_nodes = []
        visible = bool(self._cb_spheres.value)
        for name in self._sphere_links:
            if name not in geoms:
                continue
            g = geoms[name]
            centres, radii = spheres_from_primitive(
                g["type"], np.asarray(g["size"], dtype=float),
                np.asarray(g["offset"], dtype=float).reshape(4, 4), self._sphere_spacing,
            )
            for k, (c, r) in enumerate(zip(centres, radii)):
                mesh = trimesh.creation.icosphere(subdivisions=2, radius=float(r))
                mesh.visual.face_colors = _SPHERE_COLOR
                handle = self._server.scene.add_mesh_trimesh(
                    f"/collision/spheres_v2/{name}_{k}", mesh, visible=visible
                )
                self._sphere_nodes.append((handle, name, np.asarray(c, dtype=float)))
        if visible:
            self._update_spheres()

    def _update_spheres(self):
        """Place every sphere at FK(link) @ centre, using the measured arm q."""
        if not self._sphere_nodes or self._q_arm_latest is None:
            return
        cfg = np.zeros(self._n)
        cfg[: self._n_arm] = self._q_arm_latest
        self._fk.update_cfg(cfg)
        poses = {}
        for handle, link, c in self._sphere_nodes:
            if link not in poses:
                poses[link] = np.asarray(self._fk.get_transform(link), dtype=float)
            T = poses[link]
            handle.position = T[:3, :3] @ c + T[:3, 3]

    # ---------------------------------------------------------------- cause 1
    def _update_ghost_and_lag(self):
        c = self._last_ctrl
        if c is None:
            return
        t_c, q_des, qd_des, _, _, _ = c.get_data()
        q_used = q_des - self._kp_mult * qd_des * self._ctrl_dt
        if self._cb_ghost.value:
            self._ghost.update_cfg(q_used)
        self._set_ghost_visible(bool(self._cb_ghost.value))
        valid = ~np.isnan(self._buf_t)
        if valid.sum() < 10:
            return
        bt, bq = self._buf_t[valid], self._buf_q[valid]
        err = np.linalg.norm(bq - q_used[: self._n_arm], axis=1)
        e_min = float(err.min())
        if e_min > 1e-4:
            self._lag_ms = None  # older than the buffer (or not found)
            return
        ok = err <= max(1e-6, 1.5 * e_min)
        i_now = int(np.argmax(bt))  # newest measured sample = "now"
        t_now = float(bt[i_now])
        if ok[i_now]:
            # the used state equals the current one: robot at rest, age not observable
            self._lag_still = True
            self._lag_ms, self._lag_in_ms, self._lag_out_ms = 0.0, None, None
            return
        self._lag_still = False
        t_match = float(bt[ok].max())
        self._lag_ms = 1e3 * (t_now - t_match)  # total: state behind the applied command
        self._lag_in_ms = 1e3 * (t_c - t_match)  # joint_meas FIFO in the controller
        self._lag_out_ms = 1e3 * (t_now - t_c)  # joint_ctrl FIFO out of the controller
        self._lag_hist.append(self._lag_ms)

    # ---------------------------------------------------------------- cause 2/3
    def _update_witness(self):
        show_w = bool(self._cb_witness.value)
        show_v = bool(self._cb_vstar.value)
        d = self._last_dbg
        fresh = d is not None and time.time() - self._last_dbg_wall < 1.0
        if not fresh or not d["rows"]:
            self._pts.visible = False
            self._obs_lines.visible = False
            self._trail_lines.visible = False
            self._arrows.visible = False
            return
        rows = [r for r in d["rows"].values() if "p" in r and "o" in r and "m" in r]
        P = np.array([r["p"] for r in rows], dtype=np.float32)
        O = np.array([r["o"] for r in rows], dtype=np.float32)
        act = np.array([r["m"][1] for r in rows])
        cols = _act_color(act)

        self._pts.points = P
        self._pts.colors = cols
        self._pts.visible = show_w
        self._obs_lines.points = np.stack([P, O], axis=1)
        self._obs_lines.colors = np.repeat(cols[:, None, :], 2, axis=1)
        self._obs_lines.visible = show_w

        segs, scol = [], []
        for tr in self._trails.values():
            pts = [p for _, p in tr]
            for a, b in zip(pts[:-1], pts[1:]):
                segs.append((a, b))
                scol.append(_JUMP_COLOR if np.linalg.norm(b - a) > self._jump_thr else _TRAIL_COLOR)
        if segs:
            self._trail_lines.points = np.asarray(segs, dtype=np.float32)
            self._trail_lines.colors = np.repeat(np.asarray(scol)[:, None, :], 2, axis=1)
            self._trail_lines.visible = show_w
        else:
            self._trail_lines.visible = False

        V = np.array([r.get("v", np.zeros(3)) for r in rows])
        speed = np.linalg.norm(V, axis=1)
        keep = speed > 1e-3
        if show_v and keep.any():
            A = np.stack([P[keep], P[keep] + self._arrow_scale * V[keep]], axis=1)
            self._arrows.points = A.astype(np.float32)
            self._arrows.colors = np.tile(_VSTAR_COLOR, (int(keep.sum()), 1))
            self._arrows.visible = True
        else:
            self._arrows.visible = False

    def _update_plots(self):
        if len(self._pt) < 2:
            return
        t = np.asarray(self._pt, dtype=float)
        t = t - t[-1]
        Q = np.asarray(self._pq, dtype=float)
        lim = np.full_like(t, self._qd_limit)
        self._plot_qd.data = (t, Q[:, 0], Q[:, 1], Q[:, 2], lim, -lim)
        self._plot_alpha.data = (t, np.asarray(self._pa, dtype=float))

    def _update_markdown(self):
        lines = []
        d = self._last_dbg if (self._last_dbg is not None and time.time() - self._last_dbg_wall < 1.0) else None
        if d is not None and "hdr" in d:
            st = STATE_NAME.get(int(round(d["hdr"][0])), "?")
            fix = d.get("fix", np.zeros(3))
            lines.append(f"**state:** `{st}`  gain target α=`{d['hdr'][1]:.2f}`  ")
            lines.append("**fixes (fresh/multi-pt/smooth):** "
                         + " / ".join("on" if f > 0.5 else "OFF" for f in fix) + "  ")
        elif self._last_ctrl is not None:
            kd0 = self._last_ctrl.get_data()[5][0]
            lines.append(f"**gains:** `{'ON' if kd0 > 0 else 'off (grav_comp)'}`  "
                         "_(no planner debug data: original controller, or v2 not running)_  ")
        if self._lag_ms is None:
            lag = "> buffer (very old) or unmatched" if self._last_ctrl is not None else "—"
            lines.append(f"**1) applied command uses the joint state from:** `{lag}`  ")
        elif self._lag_still:
            lines.append("**1) applied command uses the joint state from:** "
                         "`— (robot at rest: move it to measure)`  ")
        else:
            lines.append(f"**1) applied command uses the joint state from:** `{self._lag_ms:.0f} ms ago`  ")
            lines.append(f"&nbsp;&nbsp;&nbsp;= meas FIFO in `{self._lag_in_ms:.0f} ms` + "
                         f"cmd FIFO out `{self._lag_out_ms:.0f} ms` (ghost = that state)  ")
        if d is not None and "stat" in d:
            lines.append(f"**2) witness jumps (last 1 s):** `{int(d['stat'][0])}`  ")
            lines.append(f"**3) max |v\\*|:** `{d['stat'][1]:.2f} m/s`  "
                         f"max |qd_des|: `{d['stat'][2]:.2f} rad/s`  ")
        if self._last_ctrl is not None:
            qd = self._last_ctrl.get_data()[2][: self._n_arm]
            sat = [str(i + 1) for i in range(self._n_arm) if abs(qd[i]) >= self._qd_limit - 1e-3]
            lines.append(f"**3) joints at the qd limit:** `{', '.join(sat) if sat else 'none'}`")
        self._md.content = "\n".join(lines)

    # ------------------------------------------------------------------ main
    def update(self):
        self._poll_meas()
        self._poll_ctrl()
        self._poll_debug()
        self._poll_col_info()
        if self._cb_spheres.value:
            self._update_spheres()
        self._update_ghost_and_lag()
        self._update_witness()
        self._update_plots()
        self._update_markdown()
