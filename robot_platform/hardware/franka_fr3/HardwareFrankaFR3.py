import os
import sys
import copy
import time
import signal
import atexit
import subprocess

import numpy as np
from omegaconf import DictConfig, OmegaConf

from robot_platform.hardware.BaseHardwarePlatform import BaseHardwarePlatform
from robot_platform.hardware.franka_fr3.Robotiq2F85Hardware import Robotiq2F85Hardware
from data_type.basic_types.ColInfoData import ColInfoData
from data_type.basic_types.JointCtrlData import JointCtrlData
from data_type.basic_types.JointMeasData import JointMeasData
from utils.collision.fr3_robotiq_geoms import build_robot_collision_primitives
from utils.mode import HandControlMode


_NUM_ARM_JOINTS = 7
_NUM_HAND_JOINTS = 1
_NUM_JOINTS = _NUM_ARM_JOINTS + _NUM_HAND_JOINTS


class HardwareFrankaFR3(BaseHardwarePlatform):
    """Combined Franka Research 3 arm + Robotiq 2F-85 hardware platform (the
    DROID setup).

    Split by how each subsystem is actuated:

    - **Arm**: handled entirely by the C++ Franka bridge (libfranka), spawned
      here as a subprocess. The bridge does its own LCM: it subscribes to the
      unified command channel, slices the arm portion
      (mapping.ctrl_joint_offset=0, num_arm_joints=7), computes torque, streams
      it to the robot from libfranka's 1 kHz control callback, and publishes
      the arm joint_meas. When it gets no fresh command it ends the torque
      control loop, so the robot holds its current pose (Idle).

    - **Gripper**: driven here in Python via Robotiq2F85Hardware (Modbus RTU
      over the gripper's USB adapter). We read the same unified 8-DoF command
      off ``ctrl_sub_que``, use the gripper slice [7] (driver angle q_des; the
      2F-85 is position controlled, so qd / tau / kp / kd are ignored, as in
      the sim), command the gripper, read its state, and publish the gripper
      joint_meas. Without a fresh command the gripper holds its last target
      (it never opens on its own, so a grasped object is not dropped).

    So the controller publishes ONE 8-DoF command ([arm(7), gripper(1)]) and
    the bridge + this platform each take their slice - matching the sim's
    contract.

    Collision info: the collision-aware controllers (tutorials 05 / 06) wait
    for the robot / static collision geometry that the sim env publishes. With
    a ``col_info`` section this platform publishes the same thing: the robot
    primitives (utils/collision/fr3_robotiq_geoms.py) and the static
    obstacles of ``col_info.scene_xml_path`` (floor / wall_* / table* / obs_*
    geoms). Those must describe YOUR real cell: they are what keeps the arm
    away from it.

    Mock mode: ``hand.mock_mode`` (default: the bridge config's
    ``robot.mock_mode``) replaces the gripper by a simulated one behind the
    same Modbus protocol, so with a mock bridge the whole platform runs with
    no robot and no gripper connected.

    ``read_only`` (dry-run) mode: the gripper is read and published but never
    commanded, and commands are ignored. Combined with a bridge config whose
    ctrl channel nothing publishes to, this makes the whole robot a pure
    sensor.
    """

    def __init__(self, config: DictConfig, *args, **kwargs):
        super().__init__(config, *args, **kwargs)

        self._read_only = bool(config.get("read_only", False))

        # ---------------- Arm (bridge subprocess) ----------------
        arm_cfg = config["arm"]
        self._bridge_binary = arm_cfg["bridge_binary"]
        self._bridge_config = arm_cfg["bridge_config"]
        self._bridge_ld_lib_path = os.path.expanduser(
            arm_cfg.get("bridge_ld_library_path", "~/libfranka/build")
        )
        self._bridge_proc = None

        # ---------------- Gripper (Robotiq 2F-85) ----------------
        hand_cfg = config["hand"]
        hand_mode_str = hand_cfg.get("control_mode", "POSITION_CONTROL")
        try:
            self._hand_control_mode = HandControlMode[hand_mode_str]
        except KeyError:
            raise ValueError(f"[Hardware] Invalid hand control mode: {hand_mode_str}")
        if self._hand_control_mode != HandControlMode.POSITION_CONTROL:
            raise ValueError(
                f"[Hardware] Robotiq 2F-85 only supports POSITION_CONTROL, "
                f"got {self._hand_control_mode.name}."
            )

        hand_mock = hand_cfg.get("mock_mode", None)
        if hand_mock is None:
            # Follow the bridge: one switch for the whole robot.
            hand_mock = bool(
                OmegaConf.load(self._bridge_config)["robot"].get("mock_mode", True)
            )
        self._hand_mock = bool(hand_mock)
        self._hand_hardware = Robotiq2F85Hardware(
            port=hand_cfg.get("port", "/dev/ttyUSB0"),
            mock=self._hand_mock,
            speed=hand_cfg.get("speed", 0.1),
            force=hand_cfg.get("force", 50.0),
            io_freq=hand_cfg.get("io_freq", 50.0),
            pos_tol=hand_cfg.get("pos_tol", 1),
            activate=hand_cfg.get("activate", True),
            calibrate_stroke=hand_cfg.get("calibrate_stroke", False),
        )
        self._hand_joint_state = JointMeasData(num_joints=_NUM_HAND_JOINTS)
        self._hand_joint_meas_channel = config["pub_manager"]["hand_joint_meas_channel"]

        # ---------------- Collision info (optional) ----------------
        self._robot_col_info_channel = config["pub_manager"].get("robot_col_info_channel", "")
        self._static_col_info_channel = config["pub_manager"].get("static_col_info_channel", "")
        col_cfg = config.get("col_info", None)
        self._robot_col_info_data = None
        self._static_col_info_data = None
        if col_cfg is not None and self._robot_col_info_channel:
            self._robot_col_info_data = ColInfoData(name=self._robot_col_info_channel)
            for geom in build_robot_collision_primitives():
                self._robot_col_info_data.add_col_geom(**geom)
        if col_cfg is not None and self._static_col_info_channel:
            import mujoco as mj

            from utils.collision.wall_geoms import extract_static_col_geoms

            self._static_col_info_data = ColInfoData(name=self._static_col_info_channel)
            scene = mj.MjModel.from_xml_path(col_cfg["scene_xml_path"])
            for geom in extract_static_col_geoms(scene):
                self._static_col_info_data.add_col_geom(**geom)
        self._col_info_update_dt = float(
            (col_cfg or {}).get("col_info_update_dt", 0.1)
        )
        self._last_col_info_t = -1e9

        # ---------------- Command bookkeeping ----------------
        self._last_ctrl_data = JointCtrlData(num_joints=_NUM_JOINTS)
        self._received_first_ctrl = False
        self._last_ctrl_t = time.time()
        self._ctrl_timeout_t = float(config.get("ctrl_timeout", 0.5))
        # Rate-limit hand command/publish to hardware_freq (self._update_dt).
        # _step() is called by the base loop as fast as it can spin, so without
        # this the hand meas floods the publisher's queue (which drains one
        # sample per cycle) and readers see rapidly growing latency.
        self._last_hand_t = time.time()

        atexit.register(self.shutdown)

    # ------------------------------------------------------------------
    # run_hardware.py wiring
    # ------------------------------------------------------------------

    def set_sub_que_dict(self, sub_data_queue_dict):
        # We DO need the command queue (for the hand slice); the bridge takes
        # the arm slice independently off its own LCM subscription.
        self.ctrl_sub_que = sub_data_queue_dict["ctrl_sub_que"]

    def set_pub_que_dict(self, pub_data_queue_dict):
        self.intr_pub_que_dict = pub_data_queue_dict["intr_pub_que_dict"]
        self.extr_pub_que_dict = pub_data_queue_dict["extr_pub_que_dict"]

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self):
        print(
            f"[Hardware] Starting Robotiq 2F-85{' (MOCK)' if self._hand_mock else ''}..."
        )
        self._hand_hardware.start()
        if self._read_only:
            # Dry-run: leave the gripper un-commanded. With no command() call the
            # IO thread never writes a target, so the fingers stay where they
            # are while still being READ.
            print("[Hardware] read_only: gripper left un-commanded (sensor-only).")

        print("[Hardware] Spawning Franka bridge...")
        self._spawn_bridge()
        # The bridge first moves the arm to its start pose (robot.move_to_start)
        # and only then starts streaming; report Ready (and accept commands)
        # once its first arm measurement arrives.
        self._wait_for_bridge()

        self._hardware_ready = True
        print(
            f"[Hardware] Ready (bridge pid={self._bridge_proc.pid}"
            f"{', READ_ONLY' if self._read_only else ''}). "
            f"Press 'q' (or Ctrl-C) to shut down."
        )

    def start(self):
        try:
            super().start()
        finally:
            self.shutdown()

    def shutdown(self):
        # Bring the bridge down first: it ends the torque control loop, so the
        # robot's own controller holds the arm (Idle).
        if self._bridge_proc is not None and self._bridge_proc.poll() is None:
            print("[Hardware] Sending SIGINT to bridge...")
            try:
                self._bridge_proc.send_signal(signal.SIGINT)
                self._bridge_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("[Hardware] Bridge didn't exit; SIGTERM.")
                self._bridge_proc.terminate()
                try:
                    self._bridge_proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    print("[Hardware] Bridge still alive; SIGKILL.")
                    self._bridge_proc.kill()
        self._bridge_proc = None

        try:
            self._hand_hardware.stop()
            print("[Hardware] Gripper stopped.")
        except Exception as e:
            print(f"[Hardware] Error stopping gripper: {e}")

    def estop(self):
        # Full shutdown: bridge down (robot holds the arm in Idle), gripper stopped.
        print("[Hardware] ESTOP.")
        self._finished = True
        self.shutdown()

    # ------------------------------------------------------------------
    # Main-loop hooks
    # ------------------------------------------------------------------

    def _step(self):
        # 1) Watch the bridge subprocess.
        if self._bridge_proc is not None and self._bridge_proc.poll() is not None:
            print(
                f"[Hardware] Bridge exited unexpectedly "
                f"(rc={self._bridge_proc.returncode}); shutting down."
            )
            self._finished = True
            return

        # 2) Drain the latest unified command.
        if not self.ctrl_sub_que.empty():
            while not self.ctrl_sub_que.empty():
                self._last_ctrl_data = self.ctrl_sub_que.get()
            self._received_first_ctrl = True
            self._last_ctrl_t = time.time()

        # Rate-limit the gripper command + publish to hardware_freq. (Draining
        # the command queue above stays every-tick so the latest command is used.)
        if time.time() - self._last_hand_t < self._update_dt:
            return
        self._last_hand_t = time.time()

        # 3) Command the gripper (skipped entirely in read-only/dry-run). Only a
        #    fresh command moves it; otherwise it holds its last target.
        if not self._read_only:
            fresh = self._received_first_ctrl and (
                time.time() - self._last_ctrl_t < self._ctrl_timeout_t
            )
            if fresh:
                _, q_des, qd_des, tau_ff, kp, kd = self._last_ctrl_data.get_data()
                self._hand_hardware.command(
                    q_des[_NUM_ARM_JOINTS:],
                    qd_des[_NUM_ARM_JOINTS:],
                    tau_ff[_NUM_ARM_JOINTS:],
                    kp[_NUM_ARM_JOINTS:],
                    kd[_NUM_ARM_JOINTS:],
                )

        # 4) Read + publish the gripper state. (Arm meas is published by the bridge.)
        q, qd, tau = self._hand_hardware.get_motor_data()
        self._hand_joint_state.set_data(time.time(), q, qd, tau)
        self.intr_pub_que_dict[self._hand_joint_meas_channel].put(
            copy.deepcopy(self._hand_joint_state)
        )

        # 5) Collision info, re-sent periodically for late-joining controllers.
        now = time.time()
        if now - self._last_col_info_t > self._col_info_update_dt:
            self._last_col_info_t = now
            if self._robot_col_info_data is not None:
                self._robot_col_info_data.set_time(now)
                self.intr_pub_que_dict[self._robot_col_info_channel].put(
                    copy.deepcopy(self._robot_col_info_data)
                )
            if self._static_col_info_data is not None:
                self._static_col_info_data.set_time(now)
                self.extr_pub_que_dict[self._static_col_info_channel].put(
                    copy.deepcopy(self._static_col_info_data)
                )

    def _handle_key(self, key):
        if key == "q":
            print("[Hardware] Quit requested.")
            self._finished = True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _wait_for_bridge(self) -> None:
        import lcm

        bridge_cfg = OmegaConf.load(self._bridge_config)
        channel = bridge_cfg["lcm"]["joint_meas_channel"]
        got = []
        lc = lcm.LCM()
        sub = lc.subscribe(channel, lambda _ch, _data: got.append(True))
        print(
            f"[Hardware] Waiting for the bridge (start-pose move, then '{channel}')..."
        )
        try:
            while not got:
                if self._bridge_proc.poll() is not None:
                    raise RuntimeError(
                        f"Franka bridge exited during start-up "
                        f"(rc={self._bridge_proc.returncode}); see its output above."
                    )
                lc.handle_timeout(100)
        finally:
            lc.unsubscribe(sub)

    def _spawn_bridge(self) -> None:
        env = os.environ.copy()
        existing = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = self._bridge_ld_lib_path + (
            ":" + existing if existing else ""
        )
        self._bridge_proc = subprocess.Popen(
            [self._bridge_binary, self._bridge_config],
            env=env,
            stdout=sys.stdout,
            stderr=sys.stderr,
        )
