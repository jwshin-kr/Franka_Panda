"""Collision-aware gravity compensation, v2: fixes for the "robot goes wild" issue.

The original controller (``CollisionAwareGravCompControl.py``) is left untouched.
This class inherits from it and only overrides what is needed for three
measured causes. Each fix can be switched off in the YAML (``v2.fix_*``), which
reproduces the original behaviour exactly while still publishing the debug
channel, so every cause can be seen in the v2 visualizer.

Cause 1: stale data in both directions (``fix_fresh_state``)
    * joint_meas in: the original pops ONE message per loop iteration from an
      unbounded FIFO. The main loop only iterates about as fast as messages
      arrive (~1 kHz, GIL contention), so the backlog made during start-up
      never drains and keeps growing: planner, g(q) and PD use a joint state
      that is hundreds of ms old.
    * joint_ctrl out: ``ctrl_pub_que`` is also an unbounded FIFO, emptied one
      command per publisher-thread loop (< 1 kHz under GIL contention), so the
      command the env applies was computed up to seconds earlier.
    Fix: drain the measurement queues (newest sample only) and keep only the
    newest unsent command in ``ctrl_pub_que``.

Cause 2: nearest-point (witness) jumps (``fix_multi_point_witness``)
    One FCL nearest point per (link, obstacle) pair jumps end-to-end when a
    long primitive is parallel to a wall, so the QP rows (and qd_des) jump.
    Fix: cover every link primitive with a chain of spheres
    (``utils/collision/sphere_swept_v2.py``); each sphere gives a unique,
    continuous witness point and one QP row.

Cause 3: over-aggressive, discontinuous command (``fix_smooth_command``)
    * v* reaches ``repulsive_coeff`` = 8 m/s, far beyond what 2 rad/s joints
      can deliver, so the QP sits on the velocity box and flips corners.
    * gains jump 0 <-> full on every state change; qd_des steps every plan.
    * "collision" commands qd_des = 0 (no escape): a user push drives the
      arm into the wall, where it stays; pulling it out re-enters
      "repulsive" with a saturated qd_des -> a violent kick.
    Fix: bounded escape speed (``repulsive_max_speed``) along the normal only,
    rows weighted by a smooth activation, a smoothness term in the QP,
    "collision" keeps repelling, gains blended by a continuous activation,
    and qd_des slew-rate limited in the 1 kHz loop.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from queue import Empty, Full

import numpy as np
import osqp
import pinocchio as pin
from omegaconf import DictConfig
from scipy import sparse

from controller.CollisionAwareGravCompControl import (
    CollisionAwareGravCompController,
    PlanResult,
)
from data_type.basic_types.JointCtrlData import JointCtrlData
from data_type.basic_types.JointMeasData import JointMeasData
from data_type.basic_types.NamedVecListData import NamedVecListData
from utils.collision.sphere_swept_v2 import signed_distance, spheres_from_primitive
from utils.lie.kinematics import get_point_Jacobian
from utils.pinocchio.getter import get_fk_link_poses, get_link_Jacobians

STATE_CODE = {"grav_comp": 0, "repulsive": 1, "collision": 2}


@dataclass
class PlanResultV2(PlanResult):
    """PlanResult + the gain activation (0 = free float, 1 = full gains)."""

    alpha: float = 1.0


@dataclass
class WitnessRow:
    """One distance row of the planner (one witness point vs one obstacle)."""

    link: str
    obs: str
    k: int  # sphere index (always 0 for the original single-point rows)
    p: np.ndarray  # witness point on the robot (world)
    p_obs: np.ndarray  # closest point on the obstacle (world)
    n: np.ndarray  # unit normal, obstacle -> robot
    dist: float  # signed distance (FCL single-point mode: -1 when penetrating)
    J: np.ndarray  # (3, n_x) Jacobian of the point used for the row
    radius: float = 0.0
    margin: float = 0.0
    vstar: np.ndarray | None = None  # desired Cartesian velocity of the point
    act: float = 0.0  # activation in [0, 1]

    @property
    def key(self) -> str:
        return f"{self.link}|{self.obs}|{self.k}"


class CollisionAwareGravCompControllerV2(CollisionAwareGravCompController):
    """Original controller + the three fixes (see module docstring)."""

    def __init__(self, config: DictConfig, *args, **kwargs):
        super().__init__(config, *args, **kwargs)

        v2 = config.get("v2", {}) or {}
        self._fix_fresh_state = bool(v2.get("fix_fresh_state", True))
        self._fix_multi_point = bool(v2.get("fix_multi_point_witness", True))
        self._fix_smooth = bool(v2.get("fix_smooth_command", True))

        # fix 2: sphere model
        self._sphere_spacing = float(v2.get("sphere_spacing_ratio", 1.0))
        # fix 3: command shaping
        self._v_max = float(v2.get("repulsive_max_speed", 1.0))  # m/s
        self._qp_reg = float(v2.get("qp_regularization", 0.02))
        self._qp_smooth = float(v2.get("qp_smoothness", 0.05))
        self._cbf_gamma = float(v2.get("cbf_gamma", 1.0 / self._plan_horizon))
        self._qd_des_max_acc = float(v2.get("qd_des_max_acc", 20.0))  # rad/s^2
        self._gain_ramp_rate = float(v2.get("gain_ramp_rate", 10.0))  # 1/s
        self._gain_full_at = float(v2.get("gain_full_at", 0.25))  # band fraction
        self._jump_thr = float(v2.get("witness_jump_thr", 0.03))  # m (debug)

        # debug channel (plan rate). Empty -> not published.
        self._debug_channel = str(config.get("debug_channel", "") or "")
        self._debug_margin = float(v2.get("debug_margin", 1.5))  # x magin_thr

        # control-thread state (fix 3)
        self._qd_des_cmd = np.zeros(self._num_joints)
        self._alpha = 0.0
        self._alpha_target = 0.0

        # planner-thread state
        self._qd_plan_prev = np.zeros(self._num_joints)
        self._prev_witness: dict[str, np.ndarray] = {}
        self._jump_times: deque = deque()
        self._last_rows: list[WitnessRow] = []

        # measurement timestamps (cause 1 diagnostics)
        self._meas_stamp = {}
        self._link_spheres: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._obstacles: dict[str, dict] = {}

        print(
            "[ColAwareGravComp v2] fixes: "
            f"fresh_state={self._fix_fresh_state}, "
            f"multi_point_witness={self._fix_multi_point}, "
            f"smooth_command={self._fix_smooth}"
        )

    # ------------------------------------------------------------------ init
    def initialize(self) -> None:
        """Original init (FCL geoms) + sphere model and analytic obstacles."""
        super().initialize()
        # The ColInfo dicts were consumed by the parent; rebuild what v2 needs
        # from the FCL cache it created (type/size are kept on the fcl geoms).
        for name in self._col_link_names:
            entry = self._fcl_robot_col_geoms[name]
            gtype, size = _fcl_geom_type_size(entry["fcl_geom"])
            self._link_spheres[name] = spheres_from_primitive(
                gtype, size, entry["offset"], self._sphere_spacing
            )
        for name in self._obs_names:
            entry = self._fcl_static_col_geoms[name]
            gtype, size = _fcl_geom_type_size(entry["fcl_geom"])
            self._obstacles[name] = {
                "type": gtype,
                "size": size,
                "T": np.asarray(entry["offset"], dtype=np.float64),
            }
        n_sph = sum(len(r) for _, r in self._link_spheres.values())
        print(f"[ColAwareGravComp v2] sphere model: {n_sph} spheres on "
              f"{len(self._link_spheres)} links.")

    # ----------------------------------------------------- cause 1: fresh state
    def _check_and_get_data_from_que(self) -> None:
        if not self._fix_fresh_state:
            # original: pop ONE message per channel per loop iteration (FIFO)
            for joint_cfg in self._dict_joints.values():
                que = self.intr_sub_que_dict[joint_cfg["joint_meas_channel"]]
                with que.mutex:
                    head = que.queue[0] if len(que.queue) else None
                if head is not None:
                    self._meas_stamp[joint_cfg["joint_meas_channel"]] = head.get_data()[0]
            super()._check_and_get_data_from_que()
            return

        for joint_cfg in self._dict_joints.values():
            channel = joint_cfg["joint_meas_channel"]
            que = self.intr_sub_que_dict[channel]
            meas = None
            while True:  # drain -> newest sample only
                try:
                    meas = que.get_nowait()
                except Empty:
                    break
            if meas is None:
                continue
            if not isinstance(meas, JointMeasData):
                raise ValueError(f"Expected JointMeasData on '{channel}', got {type(meas)}")
            t_meas, q, qd, _ = meas.get_data()
            self._meas_stamp[channel] = t_meas
            # [DBG1] same print after the fix (queue drained, newest sample only)
            if joint_cfg["list_joint_idx"][0] == 0 and self._cur_time - getattr(self, "_dbg1_t", -9.0) >= 1.0:
                self._dbg1_t = self._cur_time
                print(f"[DBG1] t={self._cur_time:6.2f}"
                      f"  meas_que={que.qsize():4d}"
                      f"  meas_age={1e3 * (self._cur_time - t_meas):6.0f} ms"
                      f"  ctrl_que={self.ctrl_pub_que.qsize():4d}")
            with self._meas_lock:
                self._q[joint_cfg["list_joint_idx"]] = q
                self._qd[joint_cfg["list_joint_idx"]] = qd

    def set_ctrl_pub_que(self, pub_que) -> None:
        """With fix_fresh_state, unsent stale commands are dropped on put()."""
        if self._fix_fresh_state:
            pub_que = _LatestOnlyQueue(pub_que)
        super().set_ctrl_pub_que(pub_que)

    def _meas_age(self) -> float:
        """How old the joint state used by the controller is (s)."""
        stamps = list(self._meas_stamp.values())  # written by the control thread
        if not stamps:
            return 0.0
        return float(self._cur_time - min(stamps))

    # ------------------------------------------------ cause 2: witness rows
    def _rows_single_point(self, q: np.ndarray) -> list[WitnessRow]:
        """Original rows: one FCL nearest point per (link, obstacle) pair."""
        links = self._get_col_links(q)
        obstacles = self._get_col_obstacles()
        col = self._get_col_data(links, obstacles)
        rows = []
        for link_name, obs_name in self._col_pairs:
            c = col[link_name][obs_name]
            delta = c["nearest_p_link"] - c["nearest_p_obs"]
            n = delta / np.clip(np.linalg.norm(delta), 1e-4, np.inf)
            rows.append(
                WitnessRow(
                    link=link_name, obs=obs_name, k=0,
                    p=np.asarray(c["nearest_p_link"], dtype=np.float64),
                    p_obs=np.asarray(c["nearest_p_obs"], dtype=np.float64),
                    n=n, dist=float(c["dist"]), J=c["point_Jac"],
                )
            )
        return rows

    def _rows_spheres(self, q: np.ndarray) -> list[WitnessRow]:
        """v2 rows: every sphere of a monitored link vs the paired obstacle."""
        poses = get_fk_link_poses(q, self._pin_model, self._pin_data_plan, self._col_link_names)
        jacs = get_link_Jacobians(q, self._pin_model, self._pin_data_plan, self._col_link_names)
        link_idx = {name: i for i, name in enumerate(self._col_link_names)}

        cache = {}
        rows = []
        for link_name, obs_name in self._col_pairs:
            i = link_idx[link_name]
            if link_name not in cache:
                T = poses[i]
                R, p = T[:3, :3], T[:3, 3]
                centres_l, radii = self._link_spheres[link_name]
                centres_w = centres_l @ R.T + p
                J = jacs[i]
                Js = [get_point_Jacobian(R, J[3:, :], J[:3, :], c) for c in centres_l]
                cache[link_name] = (centres_w, radii, Js)
            centres_w, radii, Js = cache[link_name]
            sd, p_obs, n = signed_distance(centres_w, self._obstacles[obs_name])
            for k in range(len(radii)):
                rows.append(
                    WitnessRow(
                        link=link_name, obs=obs_name, k=k,
                        p=centres_w[k] - radii[k] * n[k],  # sphere surface point
                        p_obs=p_obs[k], n=n[k],
                        dist=float(sd[k] - radii[k]),
                        # d(dist)/dt = n . (velocity of the sphere centre)
                        J=Js[k], radius=float(radii[k]),
                    )
                )
        return rows

    # --------------------------------------------------------------- planning
    def _compute_plan(self, q: np.ndarray, qd: np.ndarray) -> PlanResult:
        rows = self._rows_spheres(q) if self._fix_multi_point else self._rows_single_point(q)
        if self._fix_smooth:
            result = self._plan_smooth(q, rows)
        else:
            result = self._plan_original(q, qd, rows)
        self._last_rows = rows
        # [DBG2] same checks on the sphere rows and on qd_des + summary every 1 s
        prev_p = getattr(self, "_dbg2_p", {})
        for r in rows:
            if r.key in prev_p and r.dist < 0.10:
                step = float(np.linalg.norm(r.p - prev_p[r.key]))
                self._dbg2_max = max(getattr(self, "_dbg2_max", 0.0), step)
                if step > 0.03:
                    self._dbg2_jumps = getattr(self, "_dbg2_jumps", 0) + 1
                    print(f"[DBG2] t={self._cur_time:6.3f}  {r.key}  jump={step:.3f} m")
        self._dbg2_p = {r.key: r.p.copy() for r in rows}
        if result.state != "grav_comp":
            prev = getattr(self, "_dbg2_qd", None)
            if prev is not None:
                flip = (prev[:7] * result.qd_des[:7] < 0) & (np.abs(result.qd_des[:7] - prev[:7]) > 1.0)
                self._dbg2_flips = getattr(self, "_dbg2_flips", 0) + int(flip.sum())
            self._dbg2_qd = result.qd_des.copy()
            if self._cur_time - getattr(self, "_dbg2_t", -9.0) >= 1.0:
                self._dbg2_t = self._cur_time
                print(f"[DBG2] t={self._cur_time:6.2f}  last 1 s: witness jumps={getattr(self, '_dbg2_jumps', 0)}"
                      f"  qd_des flips={getattr(self, '_dbg2_flips', 0)}"
                      f"  max witness step={getattr(self, '_dbg2_max', 0.0):.3f} m")
                self._dbg2_jumps = self._dbg2_flips = 0
                self._dbg2_max = 0.0
        self._publish_debug(rows, result)
        return result

    # --- original command logic, written on generic rows (fix 3 OFF) -------
    def _plan_original(self, q, qd, rows: list[WitnessRow]) -> PlanResultV2:
        is_healthy, is_nearby = True, False
        for r in rows:
            delta = r.p - r.p_obs
            r.margin = r.dist - self._safety_dist_thr
            if r.margin < 0:
                weight = 0.0
                is_healthy = False
            elif r.margin < self._margin_thr:
                weight = self._repulsive_coeff * (r.margin / self._margin_thr - 1.0) ** 2
                is_nearby = True
            else:
                weight = 0.0
            r.act = 1.0 if r.margin < self._margin_thr else 0.0
            r.vstar = delta / np.clip(np.linalg.norm(delta), 1e-4, np.inf) * weight

        if not is_healthy:
            print(f"[Time: {self._cur_time}] Robot in collision, stop")
            return PlanResultV2("collision", np.zeros(self._n_x), False, 1.0)
        if not is_nearby:
            return PlanResultV2("grav_comp", np.zeros(self._n_x), True, 0.0)

        list_A, list_lb, list_ub = [], [], []
        P_np = 0.01 * np.eye(self._n_x)
        g_np = np.zeros(self._n_x)
        for r in rows:
            list_A.append((r.p - r.p_obs).T @ r.J / r.dist)
            list_lb.append((-r.dist + self._safety_dist_thr) / self._plan_horizon)
            list_ub.append(1000.0 / self._plan_horizon)
            P_np += 2.0 * r.J.T @ r.J
            g_np += -2.0 * r.J.T @ r.vstar
        qd_des, ok = self._solve_qp(
            q, P_np, g_np, np.vstack(list_A), np.array(list_lb), np.array(list_ub),
            warm=qd, ensure_zero_feasible=False,
        )
        return PlanResultV2("repulsive", qd_des, ok, 1.0)

    # --- v2 command logic (fix 3 ON) ----------------------------------------
    def _plan_smooth(self, q, rows: list[WitnessRow]) -> PlanResultV2:
        d_band = self._margin_thr
        active = []
        min_margin = np.inf
        for r in rows:
            r.margin = r.dist - self._safety_dist_thr
            min_margin = min(min_margin, r.margin)
            x = float(np.clip(1.0 - r.margin / d_band, 0.0, 1.0))  # 0 at band edge
            r.act = x
            r.vstar = self._v_max * x * x * r.n  # bounded, C1 at the band edge
            if x > 0.0:
                active.append(r)

        if not active:
            self._qd_plan_prev = np.zeros(self._n_x)
            return PlanResultV2("grav_comp", np.zeros(self._n_x), True, 0.0)

        state = "collision" if min_margin < 0.0 else "repulsive"
        alpha = float(min(1.0, max(r.act for r in active) / self._gain_full_at))

        n_x = self._n_x
        P_np = 2.0 * (self._qp_reg + self._qp_smooth) * np.eye(n_x)
        g_np = -2.0 * self._qp_smooth * self._qd_plan_prev
        A_rows, lb = [], []
        for r in active:
            a = r.n @ r.J  # normal velocity row: d(dist)/dt = a . qd
            w = r.act * r.act  # smooth weight (0 at the band edge)
            v_n = self._v_max * r.act * r.act
            P_np += 2.0 * w * np.outer(a, a)
            g_np += -2.0 * w * v_n * a
            A_rows.append(a)
            # CBF: approach no faster than gamma * margin; never approach once
            # the safety distance is violated (always feasible: qd = 0 works)
            lb.append(-self._cbf_gamma * max(r.margin, 0.0))
        A_np = np.vstack(A_rows)
        lb = np.asarray(lb)
        ub = np.full_like(lb, 1e3)

        qd_des, ok = self._solve_qp(
            q, P_np, g_np, A_np, lb, ub, warm=self._qd_plan_prev,
            ensure_zero_feasible=True,
        )
        if not ok:
            qd_des = 0.5 * self._qd_plan_prev  # decay instead of a step to zero
        self._qd_plan_prev = qd_des.copy()
        # [DBG3] bounded escape target, QP output, gain target (10 Hz)
        if self._cur_time - getattr(self, "_dbg3_t", -9.0) >= 0.1:
            self._dbg3_t = self._cur_time
            vmax = max(float(np.linalg.norm(r.vstar)) for r in active)
            sat = np.where(np.abs(qd_des[:7]) >= self._joint_velocity_limit[:7] - 1e-3)[0] + 1
            print(f"[DBG3] t={self._cur_time:6.2f}  state={state}  |v*|max={vmax:5.2f} m/s"
                  f"  wrist qd_des={np.round(qd_des[4:7], 2)}  at limit={sat}  alpha*={alpha:.2f}")
        return PlanResultV2(state, qd_des, ok, alpha)

    def _solve_qp(self, q, P_np, g_np, A_np, lb, ub, warm, ensure_zero_feasible):
        n_x = self._n_x
        l_box = np.maximum(-self._joint_velocity_limit, (self._q_lower_limit - q) / self._plan_horizon)
        u_box = np.minimum(self._joint_velocity_limit, (self._q_upper_limit - q) / self._plan_horizon)
        if ensure_zero_feasible:
            l_box = np.minimum(l_box, 0.0)
            u_box = np.maximum(u_box, 0.0)
        A = sparse.csc_matrix(np.vstack([np.eye(n_x), A_np]))
        l = np.hstack([l_box, lb])
        u = np.hstack([u_box, ub])
        if np.any(u < l):
            print("u < l, no solution")
            return np.zeros(n_x), False
        solver = osqp.OSQP()
        solver.setup(P=sparse.csc_matrix(P_np), q=g_np, A=A, l=l, u=u, verbose=False)
        solver.warm_start(x=np.asarray(warm, dtype=np.float64).copy(), y=np.zeros(A.shape[0]))
        res = solver.solve()
        if res.info.status_val == osqp.constant("OSQP_SOLVED"):
            return np.asarray(res.x, dtype=np.float64).copy(), True
        print(f"[Time: {self._cur_time}] QP status: {res.info.status}")
        return np.zeros(n_x), False

    # ------------------------------------------------------- plan -> control
    def _consume_latest_plan(self) -> None:
        latest = None
        while True:
            try:
                latest = self._plan_result_que.get_nowait()
            except Empty:
                break
        if latest is None:
            return
        if latest.state != self._last_logged_state:
            print(f"[Time: {self._cur_time}] State changed from "
                  f"{self._last_logged_state} to {latest.state}")
            self._last_logged_state = latest.state
        self._ctrl_state = latest.state
        self._qd_des = latest.qd_des.copy()
        self._alpha_target = float(getattr(latest, "alpha", 1.0))

    def _update(self) -> None:
        if not self._fix_smooth:
            super()._update()  # original: hard gain switch, qd_des steps
            return

        self._consume_latest_plan()
        with self._meas_lock:
            q = self._q.copy()
            qd = self._qd.copy()

        # slew-rate limit qd_des (no 100 Hz steps, no instant sign flips)
        max_dv = self._qd_des_max_acc * self._ctrl_dt
        self._qd_des_cmd += np.clip(self._qd_des - self._qd_des_cmd, -max_dv, max_dv)
        # ramp the gain activation instead of switching 0 <-> 1
        max_da = self._gain_ramp_rate * self._ctrl_dt
        self._alpha += float(np.clip(self._alpha_target - self._alpha, -max_da, max_da))
        # [DBG3] gain ramp instead of a switch (10 Hz while gains are on)
        if self._alpha > 0 and self._cur_time - getattr(self, "_dbg3_u", -9.0) >= 0.1:
            self._dbg3_u = self._cur_time
            print(f"[DBG3] t={self._cur_time:6.2f}  alpha={self._alpha:.2f}  kd[0]={self._alpha * self._kd[0]:.1f}")

        qd_des = self._qd_des_cmd.copy()
        q_des = q + self._kp_multiplier * qd_des * self._ctrl_dt
        tau_ff = pin.computeGeneralizedGravity(self._pin_model, self._pin_data_ctrl, q)
        tau_ff[:7] += -0.01 * qd[:7]

        q_des = self._with_gripper_set_point(q_des)
        cmd = JointCtrlData(num_joints=self._num_joints)
        cmd.set_data(self._cur_time, q_des, qd_des, tau_ff,
                     self._alpha * self._kp, self._alpha * self._kd)
        self.ctrl_pub_que.put(cmd)

    # ------------------------------------------------------------ debug pub
    def _count_witness_jumps(self, rows: list[WitnessRow]) -> int:
        now = self._cur_time
        prev = self._prev_witness
        cur = {}
        for r in rows:
            if r.margin >= self._margin_thr:
                continue  # only rows that matter to the planner
            cur[r.key] = r.p
            if r.key in prev and np.linalg.norm(r.p - prev[r.key]) > self._jump_thr:
                self._jump_times.append(now)
        self._prev_witness = cur
        while self._jump_times and now - self._jump_times[0] > 1.0:
            self._jump_times.popleft()
        return len(self._jump_times)

    def _publish_debug(self, rows: list[WitnessRow], result: PlanResult) -> None:
        n_jumps = self._count_witness_jumps(rows)
        if not self._debug_channel or not self.pub_que_dict:
            return
        que = self.pub_que_dict.get(self._debug_channel)
        if que is None:
            return

        near = [r for r in rows if r.margin < self._debug_margin * self._margin_thr]
        near.sort(key=lambda r: r.margin)
        near = near[:48]
        vmax = max((float(np.linalg.norm(r.vstar)) for r in rows if r.vstar is not None), default=0.0)
        v_scale = self._v_max if self._fix_smooth else self._repulsive_coeff
        names = ["hdr", "cfg", "fix", "stat"]
        vecs = [
            [STATE_CODE.get(result.state, 3), float(getattr(result, "alpha", 1.0)), 1e3 * self._meas_age()],
            [self._safety_dist_thr, self._margin_thr, v_scale],
            [float(self._fix_fresh_state), float(self._fix_multi_point), float(self._fix_smooth)],
            [float(n_jumps), float(vmax), float(np.max(np.abs(result.qd_des[:7])))],
        ]
        for r in near:
            key = r.key
            names += [f"p|{key}", f"o|{key}", f"v|{key}", f"m|{key}"]
            vstar = r.vstar if r.vstar is not None else np.zeros(3)
            vecs += [r.p, r.p_obs, vstar, [r.margin, r.act, r.radius]]
        msg = NamedVecListData(vec_dim=3)
        msg.set_data(self._cur_time, names, np.asarray(vecs, dtype=np.float64))
        try:
            que.put_nowait(msg)
        except Full:
            try:
                que.get_nowait()
            except Empty:
                pass
            try:
                que.put_nowait(msg)
            except Full:
                pass


class _LatestOnlyQueue:
    """Producer-side view of the publisher's FIFO: ``put`` first drops the
    commands the publisher thread has not sent yet, so at most one (the newest)
    is ever pending. Everything else is forwarded to the real queue."""

    def __init__(self, que):
        self._que = que

    def put(self, item, *args, **kwargs):
        while True:
            try:
                self._que.get_nowait()
            except Empty:
                break
        self._que.put(item, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._que, name)


def _fcl_geom_type_size(geom) -> tuple[str, np.ndarray]:
    """Recover (type, size) from an FCL primitive built by ``get_fcl_geom``."""
    name = type(geom).__name__
    if name == "Box":
        return "box", np.asarray(geom.side, dtype=np.float64)
    if name == "Cylinder":
        return "cylinder", np.array([geom.radius, geom.lz], dtype=np.float64)
    if name == "Sphere":
        return "sphere", np.array([geom.radius], dtype=np.float64)
    raise ValueError(f"Unsupported FCL geometry: {name}")
