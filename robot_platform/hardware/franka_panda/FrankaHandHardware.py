import time
import threading

import numpy as np

from utils.mode import HandControlMode

# Python bindings of libfranka from panda-py (pip package "panda-python").
# Install the build that matches the robot's libfranka version: a Panda
# (system 4.x) needs libfranka 0.9.x, the same version the C++ bridge links.
from panda_py import libfranka


class FrankaHandHardware:
    """Franka Hand (2-finger parallel gripper) driver over franka::Gripper.

    Same interface as Robotis5FHardware (start / stop / command /
    get_motor_data) so HardwareFrankaPanda can use it exactly like the Flexiv
    platform uses the Robotis hand. Joint order is [finger_joint1,
    finger_joint2], each 0 (closed) .. 0.04 m (open), matching the sim.

    The Franka Hand has no current/torque interface: it only takes a target
    width (move), and move() blocks until the fingers get there. So instead of
    one IO loop that writes every tick, there are two threads:

    - read thread: read_once() -> width, split evenly into the two fingers
      (velocity by finite difference, torque not measured -> 0).
    - command thread: sends move(width, speed) only when the target width
      changed by more than width_tol, so the hand is not re-commanded every
      tick (each move() is a slow blocking motion).

    Only POSITION_CONTROL is supported; kp / kd / tau in command() are ignored
    (the gripper firmware runs its own position control).
    """

    NUM_JOINTS = 2
    MAX_WIDTH = 0.08  # m, 2 x 0.04 finger stroke (used until homing reports one)

    def __init__(
        self,
        robot_ip,
        control_mode: HandControlMode,
        speed=0.05,
        homing=False,
        width_tol=0.002,
    ):
        if control_mode != HandControlMode.POSITION_CONTROL:
            raise ValueError(
                f"[Hardware] Franka Hand only supports POSITION_CONTROL, "
                f"got {control_mode.name}."
            )
        self.control_mode = control_mode

        print(f"[Hardware] Connecting to Franka Hand at {robot_ip}...")
        self._gripper = libfranka.Gripper(robot_ip)

        self._speed = float(speed)  # m/s
        self._homing = bool(homing)
        self._width_tol = float(width_tol)  # m
        self._max_width = FrankaHandHardware.MAX_WIDTH

        self._lock = threading.Lock()

        # sensor data
        self._currents = [0.0] * self.NUM_JOINTS  # no torque sensing on the hand
        self._velocities = [0.0] * self.NUM_JOINTS
        self._positions = [0.0] * self.NUM_JOINTS

        # commanded data: target opening width (m)
        self._target_width = None
        self._new_command_available = False

        # threading
        self._stop_event = threading.Event()
        self._read_thread = None
        self._command_thread = None

    def start(self):
        # homing calibrates the finger stroke (max_width); it moves the fingers
        # fully open and closed, so it is opt-in.
        if self._homing:
            print("[Hardware] Homing Franka Hand...")
            self._gripper.homing()

        state = self._gripper.read_once()
        if state.max_width > 0.0:
            self._max_width = state.max_width
        else:
            print(
                "[Hardware] Franka Hand reports no max_width (not homed); "
                f"using {FrankaHandHardware.MAX_WIDTH} m. Set hand.homing: true "
                "to calibrate."
            )
        with self._lock:
            self._positions[:] = [state.width / 2.0] * self.NUM_JOINTS

        if self._read_thread is None:
            self._stop_event.clear()
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            self._command_thread = threading.Thread(
                target=self._command_loop, daemon=True
            )
            self._command_thread.start()

    def stop(self):
        if self._read_thread is not None:
            self._stop_event.set()
            try:
                # abort an in-flight move() so the command thread can exit
                self._gripper.stop()
            except Exception as e:
                print(f"[FrankaHandHardware] Exception while stopping: {e}")
            self._command_thread.join()
            self._read_thread.join()
            self._read_thread = None
            self._command_thread = None

    def _read_loop(self):
        last_q = None
        last_t = None
        while not self._stop_event.is_set():
            try:
                state = self._gripper.read_once()
            except Exception as e:
                print(f"[FrankaHandHardware] Exception in reading thread: {e}")
                time.sleep(0.01)
                continue

            t = time.time()
            q = np.full(self.NUM_JOINTS, state.width / 2.0)
            if last_q is not None and t > last_t:
                qd = (q - last_q) / (t - last_t)
            else:
                qd = np.zeros(self.NUM_JOINTS)
            last_q = q
            last_t = t

            with self._lock:
                self._positions[:] = q
                self._velocities[:] = qd

    def _command_loop(self):
        last_sent_width = None
        while not self._stop_event.is_set():
            with self._lock:
                target = self._target_width
                self._new_command_available = False

            if target is None or (
                last_sent_width is not None
                and abs(target - last_sent_width) < self._width_tol
            ):
                time.sleep(0.005)
                continue

            try:
                # blocks until the width is reached (or stop() aborts it)
                self._gripper.move(target, self._speed)
            except Exception as e:
                print(f"[FrankaHandHardware] Exception in command thread: {e}")
            # remember it even on failure (e.g. fingers blocked by an object)
            # so the same target is not retried every loop
            last_sent_width = target

    def command(self, q, q_dot, tau, kp, kd):
        # based on control mode selected write appropriate data
        if self.control_mode == HandControlMode.POSITION_CONTROL:
            # each finger joint is half of the opening width
            width = float(np.clip(np.sum(q), 0.0, self._max_width))
            with self._lock:
                self._target_width = width
                self._new_command_available = True

    def get_motor_data(self):
        with self._lock:
            return (
                np.array(self._positions.copy()),
                np.array(self._velocities.copy()),
                np.array(self._currents.copy()),
            )
