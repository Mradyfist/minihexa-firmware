"""
Reconstructed API stub for the frozen module `__hw` (from __hw.py).

SOURCE: recovered from MiniHexa firmware `minihexa_20250929_0x000.bin`
        (MicroPython v1.21.0, frozen bytecode v6). This module holds the core
        robot classes. `Hiwonder.py` is a thin facade that re-exports them.

RELIABILITY:
  - Class names, method names, attribute names and string constants below are
    EXACT — extracted from the firmware's qstr pool.
  - Signatures / parameter names are RECONSTRUCTED from (a) recovered arg qstrs
    (velocity, step_num, run_time, action_name, is_blocking, onoff, ...) and
    (b) real call sites in this repo's python-programming-projects examples.
  - Method *bodies* are NOT recoverable from stripped frozen bytecode, so they
    are documented, not implemented.
"""

from typing import Callable

TYPE_UART: int

def disableLowPowerAlarm() -> None:
    """Silence the periodic low-battery alarm. Called at the top of most examples."""

def startMain() -> None: ...
def Battery_power() -> float:
    """Return current battery voltage / level (recovered symbol `Battery_power`)."""

def Singleton(cls): ...   # decorator: `wrapper` closure recovered -> classic singleton

class Buzzer:
    # attrs: __freq_queue, volume, __run
    def playTone(self, freq: int, duration_ms: int, blocking: bool = False) -> None: ...
    def setVolume(self, volume: int) -> None: ...

class Sound:
    # recovered attrs/methods: cls, istart, istop, ostart, ostop, frequency
    def istart(self, frequency: int) -> None: ...
    def istop(self) -> None: ...
    def ostart(self, frequency: int) -> None: ...
    def ostop(self) -> None: ...

class IMU:
    """MPU-class IMU over I2C. Recovered register/method symbols below."""
    _address: int
    _bus: object
    pitch: float
    yaw: float
    _gyro_offset: object
    __offsetangle: object
    def WhoAmI(self) -> int: ...
    def Read_Revision(self) -> int: ...
    def Config_apply(self) -> None: ...
    def _read_byte(self, reg: int) -> int: ...
    def _read_block(self, reg: int, length: int) -> bytes: ...
    def _read_u16(self, reg: int) -> int: ...
    def _write_byte(self, reg: int, val: int) -> None: ...
    def _offset_config(self) -> None: ...
    def read_sensor_data(self) -> tuple: ...
    def Read_Raw_XYZ(self) -> tuple: ...
    def Read_XYZ(self) -> tuple: ...
    def read_gyro_data(self) -> tuple: ...
    def read_accel_data(self) -> tuple: ...
    def getPitch(self) -> float: ...   # recovered from global qstr pool
    def getRoll(self) -> float: ...    # recovered from global qstr pool

class Button:
    # attrs: __adc, button, clicked_cb, longpressed_cb, __loop_start, __loop
    Clicked: int
    Longpressed: int
    def clicked_cb(self, cb: Callable) -> None: ...
    def longpressed_cb(self, cb: Callable) -> None: ...

class Robot:
    """
    Core hexapod controller. Reachable as `Hiwonder.Robot`.
    Recovered internal state: __uart, run_status, __action_name, __action_time,
    __action_list, velocity, position, euler, __hloop, __balancing_flag,
    __global_iic_obj, __adcp.
    """

    # ---- servo calibration (see 5.1 example) ----
    def set_deviation(self, servo_id: int, value: int) -> None:
        """Set trim offset for servo `servo_id` (1..18)."""
    def download_deviation(self) -> None:
        """Persist all deviations to controller flash (do once)."""
    def read_deviation(self) -> list:
        """Read back stored deviation values."""

    # ---- locomotion / kinematics (see 5.2 examples) ----
    def go(self, velocity: list, step_num: int = 0, run_time: int = 1000) -> None:
        """
        Omnidirectional gait. velocity = [vx, vy, vw]
          vx = strafe, vy = forward/back, vw = yaw.
        step_num: number of gait cycles (-1 = continuous). run_time: ms per step.
        """
    def set_body_pose(self, xyz: list, run_time: int = 600) -> None:
        """Translate body center of gravity [x, y, z] cm, feet planted."""
    def set_body_angle(self, rpy: list, run_time: int = 600) -> None:
        """Rotate body orientation [roll, pitch, yaw] degrees."""
    def reset(self) -> None:
        """Return to neutral standing stance."""
    def multi_servo_control(self, *args) -> None:
        """Low-level direct multi-servo command (recovered symbol)."""

    # ---- action groups (.rob files; see 5.3/01) ----
    def action_run(self, action, is_blocking: bool = True) -> None:
        """Play a stored action group. Parses .rob keys: Actions/Duration/Servos.
        On failure prints 'err:start action err' / 'err:except'."""
    def action_stop(self) -> None: ...

    # ---- self-balancing (see 5.3/06) ----
    def homeostasis(self, onoff: bool) -> None:
        """Enable/disable IMU-based self-balancing (__balancing loop)."""
    def read_angle(self) -> tuple:
        """Return current body angle from IMU."""
