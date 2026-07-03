"""
Reconstructed API stub for the frozen module `Hiwonder_BLE` (from Hiwonder_BLE.py).

SOURCE: MiniHexa firmware minihexa_20250929_0x000.bin (MicroPython v1.21.0).
        Built on `ubluetooth`. Names EXACT from qstr pool; bodies not recovered.
"""

from typing import Callable

MODE_BLE_SLAVE: int
MODE_BLE_MASTER: int
MODE_BROADCAST_SLAVE: int
MODE_BROADCAST_MASTER: int

class BLE:  # exact class name not in local pool; attrs below are exact
    max_buffer_size: int
    conn_handle: int
    addr_type: int
    connected_callback: Callable
    disconnected_callback: Callable
    advertiser: object
    rec_buffer: bytes  # decoded as "UTF-8"

    def send_data(self, data) -> None: ...
    def is_connected(self) -> bool: ...
    def has_data(self) -> bool: ...
    def contains_data(self) -> bool: ...
    def read_buffer(self) -> bytes: ...
    def read_uart_cmd(self) -> str: ...
    def parse_uart_cmd(self, cmd_str: str): ...
    def clear_buffer(self) -> None: ...
    def get_mac(self) -> str: ...
