"""
Reconstructed API stub for the frozen module `Hiwonder_DEV` (from Hiwonder_DEV.py).

SOURCE: MiniHexa firmware minihexa_20250929_0x000.bin (MicroPython v1.21.0).
        Class/method/attribute/constant names are EXACT (from qstr pool);
        signatures inferred from repo examples (5.3, 5.4, 5.5). Bodies not
        recoverable from frozen bytecode.

Recovered error strings: "Unknow attribute : %s", "Sensor not connected!".
This module talks to expansion sensors over I2C (imports IIC, __hw).
"""

class DEV_Digitaltube:
    """Dot-matrix / segment LED display (wraps ledMatrix + Button). e.g. 5.3/07,08"""
    def __init__(self, pin1: int, pin2: int) -> None: ...
    def showNum(self, num) -> None: ...
    def drawStr(self, x: int, y: int, text: str) -> None: ...
    def setBrightness(self, level: int) -> None: ...
    def setLedOnOff(self, onoff) -> None: ...
    def drawBitMap(self, *args) -> None: ...

class DEV_SONAR:
    """Ultrasonic distance sensor. e.g. 5.3/08. I2C addr in `_SONAR_ADDRESS`."""
    def __init__(self) -> None: ...
    def getDistance(self) -> int:
        """Distance in mm (register `__dist_reg`)."""
    # RGB LEDs on the sonar board (recovered):
    def setRGBMode(self, mode) -> None: ...
    def setRGB(self, index: int, r: int, g: int, b: int) -> None: ...
    def setBreathingCycle(self, *args) -> None: ...
    def setRGBBreathingValue(self, *args) -> None: ...
    def startSymphony(self) -> None: ...

class DEV_TOUCH:
    """Touch sensor. e.g. 5.3/09."""
    def __init__(self, pin: int) -> None: ...
    def read(self) -> bool: ...

class DEV_IR:
    """IR obstacle sensor. e.g. 5.3/10,11. read() True == obstacle."""
    def __init__(self, pin: int) -> None: ...
    def read(self) -> bool: ...

class DEV_ESP32S3Cam:
    """ESP32-S3 AI vision camera over I2C. e.g. 5.4/03.
    Constants recovered: __ADDR, __FACE, __COLOR_1/2/3."""
    def __init__(self) -> None: ...
    def read_color(self, color_id: int):
        """Return detected color blob info, e.g. [center_x, ...] or None."""
    def read_face(self):
        """Return detected face info or None."""

class DEV_ASR:
    """Offline speech recognition module. e.g. 5.5/05.
    Constants recovered: ASR_RESULT_ADDR, ASR_SPEAK_ADDR, ASR_CMDMAND, ASR_ANNOUNCER."""
    def __init__(self) -> None: ...
    def getResult(self) -> int:
        """Return recognized command id (0 if none)."""
    def speak(self, index: int) -> None:
        """Play a built-in announcement / speak a phrase."""
