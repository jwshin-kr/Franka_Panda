"""Robotiq 2F-85 driver: Modbus RTU over the gripper's USB / RS-485 adapter.

This is how the DROID setup connects the gripper (its USB adapter on the
computer that runs the robot, ``/dev/ttyUSB0``). Give your user access to the
port, e.g. ``sudo usermod -aG dialout $USER`` (log out and in). If the adapter
is plugged into another computer (e.g. the DROID NUC), bridge its port to TCP
there (``python3 -m serial.tools.tcp_serial_redirect -P 5020 /dev/ttyUSB0
115200``) and set ``port: socket://<nuc-ip>:5020``.

Protocol (Robotiq 2F-85 manual, "Control" / "Modbus RTU"):

* 115200 baud, 8N1, slave id 9, CRC-16/Modbus.
* Write 3 registers at 0x03E8 (function 0x10), 6 bytes:
  ``[ACTION, 0, 0, POSITION, SPEED, FORCE]``; ACTION bits: rACT (0) activate,
  rGTO (3) go to the requested position.
* Read 3 input registers at 0x07D0 (function 0x04), 6 bytes:
  ``[STATUS, 0, FAULT, POSITION ECHO, POSITION, CURRENT]``; STATUS bits:
  gACT (0), gGTO (3), gSTA (4-5, 3 = activated), gOBJ (6-7: 0 moving,
  1 / 2 stopped by an object while opening / closing, 3 arrived).
* POSITION 0 = open .. 255 = closed; SPEED / FORCE 0..255 span the datasheet
  ranges (20-150 mm/s, 20-235 N).

The joint interface is the same as the sim's: one joint, the driver angle
(rad, 0 = open .. 0.8 = closed). A target angle is converted to a pad width
(utils/gripper/robotiq_2f85.py) and the width to a register position, linear
between the positions the gripper reports fully open / closed.
"""

import threading
import time

import numpy as np

from utils.gripper import robotiq_2f85 as rq

SLAVE_ID = 9
REG_WRITE = 0x03E8
REG_READ = 0x07D0

OBJ_MOVING, OBJ_OPENING, OBJ_CLOSING, OBJ_NONE = 0, 1, 2, 3
# Typical register positions at the end stops (used until calibrate_stroke).
POS_OPEN, POS_CLOSED = 3, 230


def crc16(frame: bytes) -> bytes:
    """Modbus CRC-16 (poly 0xA001, init 0xFFFF), little-endian."""
    crc = 0xFFFF
    for b in frame:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


def write_frame(data: bytes) -> bytes:
    body = bytes([SLAVE_ID, 0x10, REG_WRITE >> 8, REG_WRITE & 0xFF, 0x00, 0x03, 0x06]) + data
    return body + crc16(body)


def read_frame() -> bytes:
    body = bytes([SLAVE_ID, 0x04, REG_READ >> 8, REG_READ & 0xFF, 0x00, 0x03])
    return body + crc16(body)


class SerialTransport:
    """The real port (pyserial, imported only here). ``port`` is a device
    (``/dev/ttyUSB0``) or a pyserial URL (``socket://host:port``)."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        import serial

        self._s = serial.serial_for_url(
            port, baudrate=baudrate, bytesize=8, parity="N", stopbits=1, timeout=timeout
        )

    def exchange(self, request: bytes, reply_len: int) -> bytes:
        self._s.reset_input_buffer()
        self._s.write(request)
        return self._s.read(reply_len)

    def close(self):
        self._s.close()


class MockRobotiqTransport:
    """A simulated 2F-85 behind the Modbus protocol (mock_mode): the fingers
    move toward the requested position at the commanded speed. Activation
    completes instantly; nothing is ever in the hand."""

    def __init__(self):
        self.act = self.gto = self.sta = 0
        self.req = self.speed = self.force = 0
        self.pos = float(POS_OPEN)
        self._t = time.time()
        self.writes = 0

    def _update(self):
        t = time.time()
        dt, self._t = t - self._t, t
        if not (self.act and self.gto):
            return
        width_rate = rq.SPEED_RANGE[0] + self.speed / 255.0 * (rq.SPEED_RANGE[1] - rq.SPEED_RANGE[0])
        rate = (POS_CLOSED - POS_OPEN) / rq.MAX_WIDTH * width_rate  # register units / s
        target = float(np.clip(self.req, POS_OPEN, POS_CLOSED))
        self.pos += float(np.clip(target - self.pos, -rate * dt, rate * dt))

    def _obj(self):
        if not (self.act and self.gto):
            return OBJ_NONE
        target = float(np.clip(self.req, POS_OPEN, POS_CLOSED))
        return OBJ_NONE if abs(self.pos - target) < 0.5 else OBJ_MOVING

    def exchange(self, request: bytes, reply_len: int) -> bytes:
        if crc16(request[:-2]) != request[-2:]:
            raise IOError("MockRobotiqTransport: bad CRC from the driver")
        self._update()
        if request[1] == 0x10:
            d = request[7:13]
            act, gto = d[0] & 1, (d[0] >> 3) & 1
            self.sta = 3 if act else 0
            self.act, self.gto = act, gto
            self.req, self.speed, self.force = d[3], d[4], d[5]
            self.writes += 1
            body = request[:6]
            return body + crc16(body)
        st = self.act | (self.gto << 3) | (self.sta << 4) | (self._obj() << 6)
        body = bytes([SLAVE_ID, 0x04, 0x06, st, 0, 0, self.req, int(round(self.pos)), 0])
        return body + crc16(body)

    def close(self):
        pass


class Robotiq2F85Hardware:
    """Robotiq 2F-85 with the same interface as the other hand drivers
    (start / stop / command / get_motor_data), so HardwareFrankaFR3 uses it
    like the Flexiv platform uses the Robotis hand.

    One IO thread at ``io_freq``: when the target changed by more than
    ``pos_tol`` register steps it writes a go-to (non-blocking: the gripper's
    firmware runs the motion, an object simply stops the fingers and they keep
    squeezing with ``force``), then reads the status. The joint measurement is
    the driver angle from the reported position, velocity by finite difference,
    torque not measured (0).

    The 2F-85 only takes position targets: kp / kd / qd / tau in command() are
    ignored.
    """

    NUM_JOINTS = 1

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        mock: bool = False,
        speed: float = 0.1,
        force: float = 50.0,
        io_freq: float = 50.0,
        pos_tol: int = 1,
        activate: bool = True,
        calibrate_stroke: bool = False,
        transport=None,
    ):
        if transport is not None:
            self._io = transport
        elif mock:
            print("[Hardware] Robotiq 2F-85: MOCK transport (no gripper connected).")
            self._io = MockRobotiqTransport()
        else:
            print(f"[Hardware] Connecting to Robotiq 2F-85 on {port}...")
            self._io = SerialTransport(port)

        self._speed_byte = self._byte(speed, *rq.SPEED_RANGE)
        self._force_byte = self._byte(force, *rq.FORCE_RANGE)
        self._io_dt = 1.0 / float(io_freq)
        self._pos_tol = int(pos_tol)
        self._activate = bool(activate)
        self._calibrate = bool(calibrate_stroke)
        self.pos_open, self.pos_closed = POS_OPEN, POS_CLOSED

        self._io_lock = threading.Lock()  # one Modbus transaction at a time
        self._lock = threading.Lock()

        # sensor data
        self._positions = np.zeros(self.NUM_JOINTS)
        self._velocities = np.zeros(self.NUM_JOINTS)
        self._currents = np.zeros(self.NUM_JOINTS)  # no torque sensing
        self._status = None

        # commanded target (register position), None = nothing commanded yet
        self._target_pos = None

        self._stop_event = threading.Event()
        self._thread = None

    # ------------------------------------------------------------ registers
    @staticmethod
    def _byte(value, lo, hi) -> int:
        return int(round(255 * (float(np.clip(value, lo, hi)) - lo) / (hi - lo)))

    def _write(self, act=1, gto=1, pos=0):
        data = bytes([(act & 1) | (gto & 1) << 3, 0, 0, int(pos), self._speed_byte, self._force_byte])
        with self._io_lock:
            reply = self._io.exchange(write_frame(data), 8)
        if len(reply) != 8 or reply[:2] != bytes([SLAVE_ID, 0x10]) or crc16(reply[:6]) != reply[6:]:
            raise IOError(f"Robotiq: bad write reply {reply.hex()}")

    def read_status(self) -> dict:
        with self._io_lock:
            reply = self._io.exchange(read_frame(), 11)
        if len(reply) != 11 or reply[:3] != bytes([SLAVE_ID, 0x04, 0x06]) or crc16(reply[:9]) != reply[9:]:
            raise IOError(f"Robotiq: bad read reply {reply.hex()}")
        st, flt, pr, po, cu = reply[3], reply[5], reply[6], reply[7], reply[8]
        return {
            "act": st & 1, "gto": (st >> 3) & 1, "sta": (st >> 4) & 3, "obj": (st >> 6) & 3,
            "fault": flt & 0x0F, "pos_request": pr, "pos": po, "current": cu,
        }

    # ------------------------------------------------------------ conversions
    def angle_to_pos(self, angle: float) -> int:
        width = float(rq.width_from_angle(np.clip(angle, 0.0, rq.MAX_ANGLE)))
        f = 1.0 - np.clip(width, 0.0, rq.MAX_WIDTH) / rq.MAX_WIDTH
        return int(round(self.pos_open + f * (self.pos_closed - self.pos_open)))

    def pos_to_angle(self, pos: int) -> float:
        f = (pos - self.pos_open) / max(self.pos_closed - self.pos_open, 1)
        width = float(np.clip((1.0 - f) * rq.MAX_WIDTH, 0.0, rq.MAX_WIDTH))
        return float(rq.angle_from_width(width))

    # ------------------------------------------------------------ lifecycle
    def _wait_until(self, cond, timeout, what):
        t0 = time.time()
        while True:
            s = self.read_status()
            if s["fault"] and s["fault"] not in (0x05,):  # 0x05: activation in progress
                raise IOError(f"Robotiq fault 0x{s['fault']:X} while {what}")
            if cond(s):
                return s
            if time.time() - t0 > timeout:
                raise TimeoutError(f"Robotiq: timeout while {what}")
            time.sleep(0.02)

    def start(self):
        s = self.read_status()
        if s["sta"] != 3:
            if not self._activate:
                raise RuntimeError(
                    "Robotiq 2F-85 is not activated; set hand.activate: true "
                    "(activation fully closes and opens the fingers once)."
                )
            print("[Hardware] Activating Robotiq 2F-85 (fingers close + open once)...")
            self._write(act=0, gto=0)
            self._write(act=1, gto=0)
            s = self._wait_until(lambda s: s["sta"] == 3, 10.0, "activating")
        if self._calibrate:
            # Measure the register positions at the two end stops.
            for attr, req in (("pos_closed", 255), ("pos_open", 0)):
                self._write(pos=req)
                s = self._wait_until(
                    lambda s, req=req: s["pos_request"] == req and s["obj"] != OBJ_MOVING,
                    5.0, "calibrating the stroke")
                if s["obj"] == OBJ_NONE:
                    setattr(self, attr, s["pos"])
                else:
                    print(f"[Hardware] Robotiq: blocked at {s['pos']} while calibrating "
                          f"(object in the gripper?); keeping {attr} = {getattr(self, attr)}")
            print(f"[Hardware] Robotiq stroke: open={self.pos_open} closed={self.pos_closed}")
        # Hold where it is until the first command (gto=0 keeps the fingers still).
        with self._lock:
            self._positions[:] = self.pos_to_angle(s["pos"])
            self._status = s

        if self._thread is None:
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._io_loop, daemon=True)
            self._thread.start()

    def stop(self):
        if self._thread is not None:
            self._stop_event.set()
            self._thread.join()
            self._thread = None
        try:
            # rGTO = 0: stop where the fingers are (an object stays gripped).
            s = self.read_status()
            self._write(gto=0, pos=s["pos"])
        except Exception as e:
            print(f"[Robotiq2F85Hardware] Exception while stopping: {e}")
        self._io.close()

    def _io_loop(self):
        last_sent = None
        last_q, last_t = None, None
        next_t = time.time()
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    target = self._target_pos
                if target is not None and (
                    last_sent is None or abs(target - last_sent) >= self._pos_tol
                ):
                    self._write(pos=target)
                    last_sent = target

                s = self.read_status()
                if s["fault"]:
                    print(f"[Robotiq2F85Hardware] fault 0x{s['fault']:X}")
                t = time.time()
                q = self.pos_to_angle(s["pos"])
                qd = (q - last_q) / (t - last_t) if last_q is not None and t > last_t else 0.0
                last_q, last_t = q, t
                with self._lock:
                    self._positions[:] = q
                    self._velocities[:] = qd
                    self._status = s
            except Exception as e:
                print(f"[Robotiq2F85Hardware] Exception in IO thread: {e}")
                time.sleep(0.05)

            next_t += self._io_dt
            delay = next_t - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.time()

    def command(self, q, q_dot=None, tau=None, kp=None, kd=None):
        """q: [driver angle] in rad (0 = open .. 0.8 = closed)."""
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if not np.all(np.isfinite(q)):
            return
        pos = self.angle_to_pos(float(q[0]))
        with self._lock:
            self._target_pos = pos

    def get_status(self):
        with self._lock:
            return None if self._status is None else dict(self._status)

    def get_motor_data(self):
        with self._lock:
            return (
                self._positions.copy(),
                self._velocities.copy(),
                self._currents.copy(),
            )
