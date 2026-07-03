"""
Reconstructed API stub for the frozen module `Hiwonder` (from Hiwonder.py).

SOURCE: MiniHexa firmware minihexa_20250929_0x000.bin (MicroPython v1.21.0).

`Hiwonder.py` is a small facade: it imports/re-exports the core classes that
actually live in `__hw.py`. Every example in this repo does `import Hiwonder`
then uses `Hiwonder.Robot()`, `Hiwonder.IMU()`, `Hiwonder.Buzzer()`,
`Hiwonder.disableLowPowerAlarm()` — all of which resolve here.

See __hw.pyi for the full method documentation of these classes.
"""

from __hw import (
    Robot as Robot,
    IMU as IMU,
    Buzzer as Buzzer,
    Sound as Sound,
    Button as Button,
    disableLowPowerAlarm as disableLowPowerAlarm,
    startMain as startMain,
    Battery_power as Battery_power,
)
