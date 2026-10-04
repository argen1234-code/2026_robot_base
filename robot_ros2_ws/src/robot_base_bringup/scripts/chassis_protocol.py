"""STM32H743 develop-branch USB CDC speed protocol (bsp_usb.c)."""

import math
import struct


def speed_frame(linear_x, angular_z, max_linear=0.20, max_angular=0.45, mode=3):
    if not all(math.isfinite(v) for v in (linear_x, angular_z)):
        raise ValueError('non-finite velocity')
    if mode not in (2, 3):
        raise ValueError('unsupported drive mode')
    if mode == 3 and linear_x < 0:
        raise ValueError('firmware indoor mode does not support reverse')
    vx = -max(-max_linear, min(linear_x, max_linear))  # MCU forward is negative Vx
    wz = max(-max_angular, min(angular_z, max_angular))
    payload = struct.pack('<Bff', mode, vx, wz)
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return b'\xaa\x55' + payload + bytes([checksum])
