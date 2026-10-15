import time

import numpy as np
import trimesh
import fcl
from yourdfpy import URDF
from viser.extras import ViserUrdf
import viser.transforms as vtf

from communication.lcm.subscriber.data_subscriber.JointMeasSubscriber import (
    JointMeasSubscriber,
)
from communication.lcm.subscriber.data_subscriber.ColInfoSubscriber import (
    ColInfoSubscriber,
)
from communication.lcm.subscriber.data_subscriber.Pose3DSubscriber import (
    Pose3DSubscriber,
)
from data_type.basic_types.JointMeasData import JointMeasData
from data_type.basic_types.ColInfoData import ColInfoData
from data_type.basic_types.Pose3DData import Pose3DData
from data_type.basic_types.ImpStatusData import (
    MODE_JOINT as IMP_MODE_JOINT,
    MODE_NAME as IMP_MODE_NAME,
    MODE_TASK as IMP_MODE_TASK,
    ImpStatusData,
)
from lcm_type.ctrl.imp_gains_cmd_t import imp_gains_cmd_t
from communication.lcm.subscriber.data_subscriber.ImpStatusSubscriber import (
    ImpStatusSubscriber,
)
from utils.fcl.getter import get_fcl_geom

# Translucent RGBA colors for the geom overlay.
_ROBOT_COL_COLOR = [80, 140, 255, 110]  # blue-ish
_WALL_COL_COLOR = [235, 140, 45, 90]  # orange-ish

# Distance-line colors (RGB).
_ENV_DIST_COLOR = (255, 40, 40)  # red: robot <-> environment
_SELF_DIST_COLOR = (255, 150, 40)  # orange: self-collision
_VEL_ARROW_COLOR = (80, 200, 255)  # cyan: desired tip velocity

_DASH_LEN = 0.012  # m
_GAP_LEN = 0.008  # m
_VEL_ARROW_SCALE = 0.4  # s: arrow length = ||v|| * scale
_VEL_ARROW_MIN = 1e-3  # m/s: hide below this
_VEL_ARROW_SHAFT = 0.004
_VEL_ARROW_HEAD_R = 0.008
_VEL_ARROW_HEAD_L = 0.02


def _primitive_mesh(geom_type, size, color):
    if geom_type == "cylinder":
        mesh = trimesh.creation.cylinder(radius=float(size[0]), height=float(size[1]))
    elif geom_type == "sphere":
        mesh = trimesh.creation.icosphere(radius=float(size[0]))
    elif geom_type == "box":
        mesh = trimesh.creation.box(extents=np.asarray(size, dtype=float))
    else:
        raise ValueError(f"Unsupported collision geom type: {geom_type}")
    mesh.visual.face_colors = color
    return mesh


def _geom_signature(dict_col_geoms):
    """Hashable summary of a col-geom set so we only rebuild meshes when the set
    (names/types/sizes) actually changes, not every message."""
    return tuple(
        (name, g["type"], tuple(np.round(np.asarray(g["size"], dtype=float), 5)))
        for name, g in sorted(dict_col_geoms.items())
    )


def _dashed_segments(p0, p1, dash_len=_DASH_LEN, gap_len=_GAP_LEN):
    """Return (N, 2, 3) dashed segments along the segment p0 -> p1."""
    p0 = np.asarray(p0, dtype=float).reshape(3)
    p1 = np.asarray(p1, dtype=float).reshape(3)
    v = p1 - p0
    length = float(np.linalg.norm(v))
    if length < 1e-9:
        return np.zeros((0, 2, 3), dtype=np.float32)
    direction = v / length
    segs = []
    t = 0.0
    while t < length:
        t1 = min(t + dash_len, length)
        segs.append([p0 + direction * t, p0 + direction * t1])
        t = t1 + gap_len
    if not segs:
        return np.zeros((0, 2, 3), dtype=np.float32)
    return np.asarray(segs, dtype=np.float32)


def _fcl_cache(dict_col_geoms):
    """Build name -> {fcl_geom, offset} from a ColInfo dict."""
    out = {}
    for name, g in dict_col_geoms.items():
        out[name] = {
            "fcl_geom": get_fcl_geom(**g),
            "offset": np.asarray(g["offset"], dtype=float).reshape(4, 4),
        }
    return out


class FrankaFR3Visualizer:
    """Renders the Franka Research 3 arm + Robotiq 2F-85 in viser, driven by two
    joint_meas_t LCM streams (arm and gripper driver angle) from the sim env.
    The URDF articulates the 2F-85 linkage from the driver angle (<mimic>).

    Same skeleton as FlexivArmHandVisualizer: the robot (+ robot cell) URDF,
    the collision overlay, the IK target overlay and the impedance overlay
    (tutorial 08: nominal / current task frames, status, gain sliders).

    A "Show collision geometry" checkbox (off by default) overlays the collision
    geometry EXACTLY as it comes off the wire on the ``robot_col_info`` and
    ``static_col_info`` LCM channels - i.e. precisely what the collision-aware
    controller receives and reasons about. The static walls carry world-frame
    offsets and are drawn as-is; the robot primitives carry link-local offsets,
    so they are placed at ``FK(link) @ offset`` using the same joint stream the
    controller uses. If those channels aren't being published (e.g. running a
    non-collision env), the overlay is simply empty.

    A "Show collision distances" checkbox draws dashed nearest-point lines for
    configured collision pairs whose FCL distance is below ``col_dist_thr``:
    red for robot <-> environment, orange for self-collision.
    """

    def __init__(
        self,
        viser_server,
        lcm_instance,
        arm_joint_meas_data_queue,
        hand_joint_meas_data_queue,
        arm_joint_meas_channel: str,
        hand_joint_meas_channel: str,
        urdf_path: str,
        mesh_dir: str = "",
        num_arm_joints: int = 7,
        num_hand_joints: int = 1,
        robot_col_info_channel: str = "",
        static_col_info_channel: str = "",
        col_pairs=None,
        self_col_pairs=None,
        col_dist_thr: float = 0.1,
        target_pose_channel: str = "",
        impedance_status_channel: str = "",
        impedance_gains_cmd_channel: str = "",
        impedance_frame_axes_length: float = 0.08,
        kq_scale: float = 1.0,
        dq_scale: float = 1.0,
        kt_trans_scale: float = 1.0,
        kt_rot_scale: float = 1.0,
        dt_trans_scale: float = 1.0,
        dt_rot_scale: float = 1.0,
        *args,
        **kwargs,
    ):
        self._viser_server = viser_server
        self._lcm_instance = lcm_instance
        self._arm_queue = arm_joint_meas_data_queue
        self._hand_queue = hand_joint_meas_data_queue

        self._num_arm_joints = num_arm_joints
        self._num_hand_joints = num_hand_joints
        self._num_joints = num_arm_joints + num_hand_joints

        self._q = np.zeros(self._num_joints)

        self._col_pairs = [tuple(p) for p in (col_pairs or [])]
        self._self_col_pairs = [tuple(p) for p in (self_col_pairs or [])]
        self._col_dist_thr = float(col_dist_thr)

        JointMeasSubscriber(
            self._lcm_instance, arm_joint_meas_data_queue, num_arm_joints
        ).subscribe(arm_joint_meas_channel)
        JointMeasSubscriber(
            self._lcm_instance, hand_joint_meas_data_queue, num_hand_joints
        ).subscribe(hand_joint_meas_channel)

        urdf_kwargs = dict(
            load_meshes=True,
            build_scene_graph=True,
            load_collision_meshes=False,
            build_collision_scene_graph=False,
        )
        if mesh_dir:
            urdf_kwargs["mesh_dir"] = mesh_dir
        urdf = URDF.load(urdf_path, **urdf_kwargs)
        self._urdf = urdf  # kept for FK of the robot collision links
        self._viser_urdf = ViserUrdf(
            self._viser_server,
            urdf_or_path=urdf,
            load_meshes=True,
            load_collision_meshes=False,
        )
        # Hidden until the first arm measurement: before that q is all zeros
        # (arm straight up, joint 4 outside its FR3 range), not the robot.
        self._viser_urdf.show_visual = False
        self._have_arm_meas = False
        self._arm_meas_channel = arm_joint_meas_channel
        self._t_start = time.monotonic()
        self._t_last_warn = self._t_start

        self._setup_collision_overlay(robot_col_info_channel, static_col_info_channel)
        self._setup_target_pose(target_pose_channel)
        # Tutorial 08: nominal vs current task frame + gain / mode tuning.
        self._setup_impedance_gui(
            status_channel=impedance_status_channel,
            gains_cmd_channel=impedance_gains_cmd_channel,
            axes_length=float(impedance_frame_axes_length),
            kq_scale=kq_scale,
            dq_scale=dq_scale,
            kt_trans_scale=kt_trans_scale,
            kt_rot_scale=kt_rot_scale,
            dt_trans_scale=dt_trans_scale,
            dt_rot_scale=dt_rot_scale,
        )

    def _setup_target_pose(self, target_pose_channel: str):
        import queue as _queue

        self._target_pose_queue = _queue.Queue()
        self._target_handle = None
        self._tip_handle = None
        self._target_vel_handle = None
        if not target_pose_channel:
            return

        Pose3DSubscriber(self._lcm_instance, self._target_pose_queue).subscribe(
            target_pose_channel
        )
        self._target_handle = self._viser_server.scene.add_icosphere(
            "/ik/target",
            radius=0.012,
            color=(80, 220, 120),
            position=(0.0, 0.0, 0.0),
            visible=False,
        )
        # Current task point (published tip); distinct from the green target.
        self._tip_handle = self._viser_server.scene.add_icosphere(
            "/ik/tip",
            radius=0.008,
            color=(255, 80, 80),
            position=(0.0, 0.0, 0.0),
            visible=False,
        )
        # Placeholder segment; updated each poll from tip → tip + scale * v*.
        self._target_vel_handle = self._viser_server.scene.add_arrows(
            "/ik/target_vel",
            points=np.zeros((1, 2, 3), dtype=np.float32),
            colors=np.asarray(_VEL_ARROW_COLOR, dtype=np.uint8),
            shaft_radius=_VEL_ARROW_SHAFT,
            head_radius=_VEL_ARROW_HEAD_R,
            head_length=_VEL_ARROW_HEAD_L,
            visible=False,
        )

    def _poll_target_pose(self):
        if self._target_handle is None:
            return
        data = None
        while not self._target_pose_queue.empty():
            data = self._target_pose_queue.get()
        if not isinstance(data, Pose3DData):
            return
        _, p_des, velocity, tip = data.get_data()
        p_des = np.asarray(p_des, dtype=float).reshape(3)
        velocity = np.asarray(velocity, dtype=float).reshape(3)
        tip = np.asarray(tip, dtype=float).reshape(3)

        self._target_handle.position = tuple(p_des)
        self._target_handle.visible = True
        self._tip_handle.position = tuple(tip)
        self._tip_handle.visible = True

        if self._target_vel_handle is None:
            return
        speed = float(np.linalg.norm(velocity))
        if speed < _VEL_ARROW_MIN:
            self._target_vel_handle.visible = False
            return
        end = tip + velocity * _VEL_ARROW_SCALE
        self._target_vel_handle.points = np.asarray(
            [[tip, end]], dtype=np.float32
        )
        self._target_vel_handle.visible = True

    def _setup_collision_overlay(self, robot_col_info_channel, static_col_info_channel):
        import queue as _queue

        # Subscribe to the SAME channels the controller consumes.
        self._robot_col_queue = _queue.Queue()
        self._static_col_queue = _queue.Queue()
        if robot_col_info_channel:
            ColInfoSubscriber(self._lcm_instance, self._robot_col_queue).subscribe(
                robot_col_info_channel
            )
        if static_col_info_channel:
            ColInfoSubscriber(self._lcm_instance, self._static_col_queue).subscribe(
                static_col_info_channel
            )

        # Live node / FCL state.
        self._robot_col_nodes = {}  # name -> (handle, link_name, offset)
        self._wall_col_nodes = {}  # name -> handle
        self._robot_col_sig = None
        self._static_col_sig = None
        self._fcl_robot = {}  # name -> {fcl_geom, offset}
        self._fcl_static = {}

        self._show_collision = False
        self._checkbox = self._viser_server.gui.add_checkbox(
            "Show collision geometry", initial_value=False
        )

        @self._checkbox.on_update
        def _(_event):
            self._show_collision = self._checkbox.value
            for handle, _ln, _off in self._robot_col_nodes.values():
                handle.visible = self._show_collision
            for handle in self._wall_col_nodes.values():
                handle.visible = self._show_collision
            if self._show_collision:
                self._update_robot_col_transforms()

        # Distance lines (batched into two handles).
        empty = np.zeros((1, 2, 3), dtype=np.float32)
        self._env_dist_handle = self._viser_server.scene.add_line_segments(
            "/collision/distances/env",
            points=empty,
            colors=_ENV_DIST_COLOR,
            line_width=2.5,
            visible=False,
        )
        self._self_dist_handle = self._viser_server.scene.add_line_segments(
            "/collision/distances/self",
            points=empty,
            colors=_SELF_DIST_COLOR,
            line_width=2.5,
            visible=False,
        )

        self._show_distances = False
        self._dist_checkbox = self._viser_server.gui.add_checkbox(
            "Show collision distances", initial_value=False
        )

        @self._dist_checkbox.on_update
        def _(_event):
            self._show_distances = self._dist_checkbox.value
            if not self._show_distances:
                self._env_dist_handle.visible = False
                self._self_dist_handle.visible = False
            else:
                self._update_collision_distances()

    def _drain_latest_col(self, q):
        data = None
        while not q.empty():
            data = q.get()
        return data if isinstance(data, ColInfoData) else None

    def _rebuild_robot_nodes(self, dict_col_geoms):
        for _h, _ln, _o in self._robot_col_nodes.values():
            _h.remove()
        self._robot_col_nodes = {}
        for name, g in dict_col_geoms.items():
            mesh = _primitive_mesh(g["type"], g["size"], _ROBOT_COL_COLOR)
            handle = self._viser_server.scene.add_mesh_trimesh(
                f"/collision/robot/{name}", mesh, visible=self._show_collision
            )
            # Robot geom name == pinocchio/URDF link frame; offset is link-local.
            self._robot_col_nodes[name] = (handle, name, np.asarray(g["offset"]))

    def _rebuild_wall_nodes(self, dict_col_geoms):
        for _h in self._wall_col_nodes.values():
            _h.remove()
        self._wall_col_nodes = {}
        for name, g in dict_col_geoms.items():
            mesh = _primitive_mesh(g["type"], g["size"], _WALL_COL_COLOR)
            T = np.asarray(g["offset"])  # world-frame SE3
            handle = self._viser_server.scene.add_mesh_trimesh(
                f"/collision/walls/{name}",
                mesh,
                wxyz=vtf.SO3.from_matrix(T[:3, :3]).wxyz,
                position=T[:3, 3],
                visible=self._show_collision,
            )
            self._wall_col_nodes[name] = handle

    def _poll_collision_info(self):
        robot = self._drain_latest_col(self._robot_col_queue)
        if robot is not None:
            _, geoms = robot.get_data()
            sig = _geom_signature(geoms)
            if sig != self._robot_col_sig:
                self._robot_col_sig = sig
                self._fcl_robot = _fcl_cache(geoms)
                self._rebuild_robot_nodes(geoms)

        static = self._drain_latest_col(self._static_col_queue)
        if static is not None:
            _, geoms = static.get_data()
            sig = _geom_signature(geoms)
            if sig != self._static_col_sig:
                self._static_col_sig = sig
                self._fcl_static = _fcl_cache(geoms)
                self._rebuild_wall_nodes(geoms)

    def _update_robot_col_transforms(self):
        """Place each robot collision primitive at FK(link) @ local_offset, using
        the same joint stream the controller uses."""
        if not self._robot_col_nodes:
            return
        self._urdf.update_cfg(self._q)
        for handle, link_name, offset in self._robot_col_nodes.values():
            T = self._urdf.get_transform(link_name) @ offset
            handle.position = T[:3, 3]
            handle.wxyz = vtf.SO3.from_matrix(T[:3, :3]).wxyz

    def _robot_fcl_objects(self):
        """World-frame FCL CollisionObjects for every known robot primitive."""
        self._urdf.update_cfg(self._q)
        objs = {}
        for name, entry in self._fcl_robot.items():
            try:
                T_link = self._urdf.get_transform(name)
            except KeyError:
                continue
            T = T_link @ entry["offset"]
            objs[name] = fcl.CollisionObject(
                entry["fcl_geom"],
                fcl.Transform(T[:3, :3], T[:3, 3]),
            )
        return objs

    def _static_fcl_objects(self):
        objs = {}
        for name, entry in self._fcl_static.items():
            T = entry["offset"]
            objs[name] = fcl.CollisionObject(
                entry["fcl_geom"],
                fcl.Transform(T[:3, :3], T[:3, 3]),
            )
        return objs

    @staticmethod
    def _pair_nearest_points(obj_a, obj_b):
        req = fcl.DistanceRequest(enable_nearest_points=True)
        res = fcl.DistanceResult()
        fcl.distance(obj_a, obj_b, req, res)
        return float(res.min_distance), res.nearest_points[0], res.nearest_points[1]

    @staticmethod
    def _apply_dashed_handle(handle, segment_batches, color, visible):
        if not segment_batches:
            handle.visible = False
            return
        points = np.concatenate(segment_batches, axis=0)
        if points.shape[0] == 0:
            handle.visible = False
            return
        handle.points = points.astype(np.float32)
        handle.colors = np.tile(
            np.asarray(color, dtype=np.uint8), (points.shape[0], 2, 1)
        )
        handle.visible = visible

    def _update_collision_distances(self):
        """Draw dashed nearest-point lines for pairs under ``col_dist_thr``."""
        if not self._show_distances:
            return
        if not self._fcl_robot:
            self._env_dist_handle.visible = False
            self._self_dist_handle.visible = False
            return

        robot_objs = self._robot_fcl_objects()
        static_objs = self._static_fcl_objects()
        thr = self._col_dist_thr

        env_segs = []
        for a, b in self._col_pairs:
            if a not in robot_objs or b not in static_objs:
                continue
            dist, p0, p1 = self._pair_nearest_points(robot_objs[a], static_objs[b])
            if dist < thr:
                env_segs.append(_dashed_segments(p0, p1))

        self_segs = []
        for a, b in self._self_col_pairs:
            if a not in robot_objs or b not in robot_objs:
                continue
            dist, p0, p1 = self._pair_nearest_points(robot_objs[a], robot_objs[b])
            if dist < thr:
                self_segs.append(_dashed_segments(p0, p1))

        self._apply_dashed_handle(
            self._env_dist_handle, env_segs, _ENV_DIST_COLOR, True
        )
        self._apply_dashed_handle(
            self._self_dist_handle, self_segs, _SELF_DIST_COLOR, True
        )

    # ------------------------------------------------------------------
    def _setup_impedance_gui(
        self,
        status_channel: str,
        gains_cmd_channel: str,
        axes_length: float,
        kq_scale: float,
        dq_scale: float,
        kt_trans_scale: float,
        kt_rot_scale: float,
        dt_trans_scale: float,
        dt_rot_scale: float,
    ):
        """Nominal vs current SE(3) frames + gain / mode tuning for impedance."""
        import queue as _queue
        import time as _time

        self._imp_status_queue = _queue.Queue()
        self._imp_gains_cmd_channel = str(gains_cmd_channel or "")
        self._imp_status_md = None
        self._imp_frame_nom = None
        self._imp_frame_cur = None
        self._last_imp_status = None
        self._suppress_imp_gains_pub = False

        if not status_channel:
            return

        ImpStatusSubscriber(self._lcm_instance, self._imp_status_queue).subscribe(
            status_channel
        )

        self._imp_frame_nom = self._viser_server.scene.add_frame(
            "/imp/T_nom",
            axes_length=axes_length,
            axes_radius=0.004,
            origin_radius=0.008,
            origin_color=(180, 80, 255),
            visible=True,
        )
        self._imp_frame_cur = self._viser_server.scene.add_frame(
            "/imp/T",
            axes_length=axes_length * 0.85,
            axes_radius=0.0035,
            origin_radius=0.007,
            origin_color=(80, 200, 120),
            visible=True,
        )

        with self._viser_server.gui.add_folder("Impedance"):
            self._imp_status_md = self._viser_server.gui.add_markdown(
                "_waiting for impedance status…_"
            )
            self._imp_mode_dropdown = self._viser_server.gui.add_dropdown(
                "Mode",
                options=["joint", "task"],
                initial_value="joint",
            )

            @self._imp_mode_dropdown.on_update
            def _(_event):
                if self._suppress_imp_gains_pub:
                    return
                self._publish_imp_gains_cmd()

            self._kq_scale_slider = self._viser_server.gui.add_slider(
                "kq_scale (joint)", min=0.0, max=3.0, step=0.05, initial_value=float(kq_scale)
            )
            self._dq_scale_slider = self._viser_server.gui.add_slider(
                "dq_scale (joint)", min=0.0, max=3.0, step=0.05, initial_value=float(dq_scale)
            )
            self._kt_trans_slider = self._viser_server.gui.add_slider(
                "kt_trans_scale",
                min=0.0,
                max=3.0,
                step=0.05,
                initial_value=float(kt_trans_scale),
            )
            self._kt_rot_slider = self._viser_server.gui.add_slider(
                "kt_rot_scale",
                min=0.0,
                max=3.0,
                step=0.05,
                initial_value=float(kt_rot_scale),
            )
            self._dt_trans_slider = self._viser_server.gui.add_slider(
                "dt_trans_scale",
                min=0.0,
                max=3.0,
                step=0.05,
                initial_value=float(dt_trans_scale),
            )
            self._dt_rot_slider = self._viser_server.gui.add_slider(
                "dt_rot_scale",
                min=0.0,
                max=3.0,
                step=0.05,
                initial_value=float(dt_rot_scale),
            )

            def _on_gain(_event=None):
                if self._suppress_imp_gains_pub:
                    return
                self._publish_imp_gains_cmd()

            for sl in (
                self._kq_scale_slider,
                self._dq_scale_slider,
                self._kt_trans_slider,
                self._kt_rot_slider,
                self._dt_trans_slider,
                self._dt_rot_slider,
            ):
                sl.on_update(_on_gain)

        # No publish here: the dropdown starts at "joint" and would switch a
        # controller running in task mode. Gains / mode are sent on user
        # changes, once the dropdown has synced to the controller's status.
        self._imp_boot_t = _time.time()

    def _publish_imp_gains_cmd(self) -> None:
        if not self._imp_gains_cmd_channel:
            return
        if not hasattr(self, "_kq_scale_slider"):
            return
        if self._last_imp_status is None:
            return  # mode dropdown not synced to the controller yet
        import time as _time

        msg = imp_gains_cmd_t()
        msg.timestamp = float(_time.time())
        msg.kq_scale = float(self._kq_scale_slider.value)
        msg.dq_scale = float(self._dq_scale_slider.value)
        msg.kt_trans_scale = float(self._kt_trans_slider.value)
        msg.kt_rot_scale = float(self._kt_rot_slider.value)
        msg.dt_trans_scale = float(self._dt_trans_slider.value)
        msg.dt_rot_scale = float(self._dt_rot_slider.value)
        mode_name = str(self._imp_mode_dropdown.value).lower()
        msg.mode = IMP_MODE_TASK if mode_name.startswith("t") else IMP_MODE_JOINT
        self._lcm_instance.publish(self._imp_gains_cmd_channel, msg.encode())

    def _poll_imp_status(self) -> None:
        if not hasattr(self, "_imp_status_queue"):
            return
        data = None
        while not self._imp_status_queue.empty():
            data = self._imp_status_queue.get()
        if not isinstance(data, ImpStatusData):
            return
        self._last_imp_status = data

        if self._imp_frame_nom is not None:
            self._imp_frame_nom.position = tuple(data.p_nom)
            self._imp_frame_nom.wxyz = tuple(data.quat_nom_wxyz)
            self._imp_frame_nom.visible = True
        if self._imp_frame_cur is not None:
            self._imp_frame_cur.position = tuple(data.p)
            self._imp_frame_cur.wxyz = tuple(data.quat_wxyz)
            self._imp_frame_cur.visible = True

        if self._imp_status_md is not None:
            mode = IMP_MODE_NAME.get(int(data.mode), str(data.mode))
            n_arm = max(1, int(data.num_arm))
            e_q = float(np.linalg.norm(data.q[:n_arm] - data.q_nom[:n_arm]))
            e_xi = float(np.linalg.norm(data.xi))
            e_p = float(np.linalg.norm(data.p - data.p_nom))
            self._imp_status_md.content = (
                f"**mode:** `{mode}`  \n"
                f"**||q−q_nom||_arm:** `{e_q:.4f}` rad  \n"
                f"**||p−p_nom||:** `{e_p:.4f}` m  \n"
                f"**||ξ|| (SE3 log):** `{e_xi:.4f}`  \n"
                f"**ξ=(ν,ω):** `[{', '.join(f'{v:.3f}' for v in data.xi)}]`  \n"
                f"**kq/dq scale:** `{data.kq_scale:.2f}` / `{data.dq_scale:.2f}`  \n"
                f"**kt (trans/rot):** `{data.kt_trans:.1f}` / `{data.kt_rot:.2f}`  \n"
                f"**dt (trans/rot):** `{data.dt_trans:.1f}` / `{data.dt_rot:.2f}`"
            )

        # Keep dropdown in sync when mode changed from the controller keyboard.
        if hasattr(self, "_imp_mode_dropdown"):
            want = "task" if int(data.mode) == IMP_MODE_TASK else "joint"
            if str(self._imp_mode_dropdown.value) != want:
                self._suppress_imp_gains_pub = True
                try:
                    self._imp_mode_dropdown.value = want
                finally:
                    self._suppress_imp_gains_pub = False

    def _drain_latest(self, queue):
        data = None
        while not queue.empty():
            data = queue.get()
        if data is not None and not isinstance(data, JointMeasData):
            raise ValueError(f"Expected JointMeasData on the queue, got {type(data)}")
        return data

    def _check_and_get_data_from_que(self):
        arm = self._drain_latest(self._arm_queue)
        if arm is not None:
            _, q, _, _ = arm.get_data()
            self._q[: self._num_arm_joints] = q
            if not self._have_arm_meas:
                self._have_arm_meas = True
                self._viser_urdf.show_visual = True
                print(f"[Visualizer] Receiving '{self._arm_meas_channel}'")
        elif not self._have_arm_meas and time.monotonic() - self._t_last_warn > 3.0:
            self._t_last_warn = time.monotonic()
            print(f"[Visualizer] No joint data on '{self._arm_meas_channel}' for "
                  f"{self._t_last_warn - self._t_start:.0f} s. sw_* = sim (run_env.py), "
                  "hw_* = real robot (run_hardware.py): use the *_hw.yaml "
                  "visualizer config with the real robot.")

        hand = self._drain_latest(self._hand_queue)
        if hand is not None:
            _, q, _, _ = hand.get_data()
            self._q[self._num_arm_joints :] = q

    def update(self):
        self._check_and_get_data_from_que()
        self._viser_urdf.update_cfg(self._q)
        # Pick up any new collision geometry off the wire.
        self._poll_collision_info()
        self._poll_target_pose()
        self._poll_imp_status()
        # Only pay the FK cost for the overlay when it's actually shown.
        if self._show_collision:
            self._update_robot_col_transforms()
        if self._show_distances:
            self._update_collision_distances()
