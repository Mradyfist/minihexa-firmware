# MiniHexa firmware analysis — `minihexa_20250929_0x000.bin`

## What the file is

- **ESP32 flashable firmware image** written at offset `0x0` (bootloader `0xE9` magic at `0x1000`).
- **MicroPython v1.21.0-dirty**, built **2025-09-29** (matches the filename), GCC 11.2.0.
- MicroPython bytecode format **v6**.
- Size: **1,945,760 bytes (0x1DB0A0)**.
- **Complete, not truncated.** The image is a concatenation of three
  self-consistent ESP sub-images:
  - bootloader @ `0x1000` — 3 segments, appended **SHA-256 verifies OK**.
  - partition table @ `0x8000`.
  - factory app @ `0x10000` — 8 segments, appended **SHA-256 verifies OK**,
    ending at exactly `0x1DB0A0` (= end of file, 0 trailing bytes).
  A matching trailing SHA-256 is proof the app image is intact.

## Partition map (table @ `0x8000`)

| Label | Type | Offset | Size | Contents in this image? |
|-------|------|--------|------|-------------------------|
| nvs | data/nvs | 0x009000 | 24 KiB | n/a (runtime NVS; not part of firmware) |
| phy_init | data/phy | 0x00F000 | 4 KiB | n/a |
| **factory** | app | 0x010000 | 2368 KiB | **yes — full app, this is what we analyze** |
| **vfs** | data/FAT | 0x260000 | 1664 KiB | intentionally empty (see below) |

> **Not a limitation / nothing is missing for reflashing.** A firmware image
> does not ship filesystem *contents*. The `vfs` (FAT) partition is created and
> **formatted by the device itself on first boot** — that's exactly what the
> frozen `inisetup.py` / `flashbdev.py` modules in this image do. So this `.bin`
> flashed at `0x0` is a complete, sufficient reflash image.
>
> Consequence for source recovery: the `vfs` only ever holds *user* files
> (e.g. your own `main.py`) — never the vendor library, which is frozen into the
> app. So even a full 4 MB dump of a used device
> (`esptool.py read_flash 0 0x400000 full.bin`) would not yield any more of the
> `Hiwonder` library than we already have here.

## The `Hiwonder` library is *frozen bytecode*, not source

The vendor Python libraries are **frozen into the `factory` app** at build time
(compiled `.py` → bytecode, embedded in the firmware). Frozen module directory
recovered from the image:

```
__hw.py   Hiwonder.py   Hiwonder_DEV.py   Hiwonder_BLE.py   ledMatrix.py
_boot.py  _boot_robot.py  inisetup.py  flashbdev.py
+ stdlib: asyncio/*, umqtt/*, webrepl, neopixel, ds18x20, onewire, dht, apa106, requests, mip, ntptime
```

`import Hiwonder` therefore loads **frozen bytecode**, not a `.py` file.

## What "decompile" can and cannot yield here

MicroPython frozen bytecode is **stripped**: it keeps the *qstr pool* (all
identifier and string-literal names) and the raw opcodes, but **not** original
source text, comments, or local-variable names. There is no reliable
bytecode→source decompiler for MicroPython. So:

- **Fully recoverable (done):** every class name, method name, attribute name,
  numeric/​string **constant name**, and **error/format string**. This
  reconstructs the *entire public API surface* with high confidence.
- **Not faithfully recoverable:** the statement-level body of each method
  (control flow, exact arithmetic, literal integer values of registers).
  Fabricating those would be guessing, so the reconstructed files document
  behavior rather than inventing implementations.

## Deliverables in this folder

- `firmware_map.txt` — partition + frozen-module map.
- `qstrs_frozen_modules.txt` — raw recovered symbol pool (exact strings).
- `recovered/*.pyi` — reconstructed, documented API stubs:
  - `Hiwonder.pyi` — facade re-exporting the core classes.
  - `__hw.pyi` — core classes: `Robot`, `IMU`, `Buzzer`, `Sound`, `Button`.
  - `Hiwonder_DEV.pyi` — expansion sensors (`DEV_SONAR`, `DEV_IR`, `DEV_TOUCH`,
    `DEV_ASR`, `DEV_ESP32S3Cam`, `DEV_Digitaltube`).
  - `Hiwonder_BLE.pyi` — BLE UART bridge.
  - `ledMatrix.pyi` — LED matrix driver.

Signatures in the `.pyi` files were cross-checked against the real call sites
in `python-programming-projects/` so they match how the examples actually use
the library.

## Reproduce

```bash
python3 -m venv .fw_venv && . .fw_venv/bin/activate
pip install esptool littlefs-python
# partition parse + qstr extraction were done with small ad-hoc python scripts
```
