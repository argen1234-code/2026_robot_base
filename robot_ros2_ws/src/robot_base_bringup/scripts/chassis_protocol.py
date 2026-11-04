"""STM32H743 USB CDC protocol (app_usb.c).

下行（Pi -> MCU）
  AA 55 | mode u8 | vx f32 | wz f32 | xor        12 字节
  BB 55 | cmd u8  | xor                          4 字节，1=ARM 2=DISARM

上行（MCU -> Pi）
  CC 55 | ver | flags | mode | seq u16 | tick_ms u32
        | cmd_vx f32 | cmd_vz f32
        | left_speed | right_speed
        | left_target | right_target
        | left_current | right_current
        | roll | pitch | yaw
        | gyro_x | gyro_y | gyro_z
        | acc_x | acc_y | acc_z
        | req_left | req_right
        | last_result u32 | rx_frames u32 | rx_bad u32 | tx_busy u32
        | xor                                    104 字节

约定：下行 vx<0 表示前进、wz<0 表示左转；上行轮速为“前进为正”的车辆坐标，
角度单位为度、陀螺为度/秒、加速度为 g。
"""

import struct

# ---- 下行 -------------------------------------------------------------------
SERVICE_ARM = 1
SERVICE_DISARM = 2

# ---- 上行 -------------------------------------------------------------------
TELEMETRY_HEADER = b'\xcc\x55'
TELEMETRY_VERSION = 2
TELEMETRY_SIZE = 104
_TELEMETRY_FORMAT = '<BBBBBHI19fIIIIB'

FLAG_IMU_READY = 1 << 0
FLAG_IMU_ONLINE = 1 << 1
FLAG_MOTORS_READY = 1 << 2
FLAG_ARMED = 1 << 3
FLAG_CMD_FRESH = 1 << 4
FLAG_FATAL = 1 << 5
FLAG_RX_SEEN = 1 << 6
FLAG_CMD_TIMEOUT = 1 << 7

# 与 app_motor.h 的 RESULT_* 保持一致（仅用于日志可读性）。
_RESULT_NAMES = {
    0: 'OK', 1: 'BAD_COMMAND', 2: 'RANGE', 3: 'STATE', 4: 'CONFIRM',
    5: 'OFFLINE', 6: 'TX', 7: 'READBACK', 8: 'MOTOR_FAULT', 9: 'TIMEOUT',
    10: 'BUSOFF', 11: 'PROTECTION', 13: 'HEALTH', 14: 'OVERSPEED',
    15: 'SERVICE_GAP', 16: 'PENDING', 17: 'BUSY',
}


def result_name(value):
    return _RESULT_NAMES.get(value, str(value))


def speed_frame(linear_x, angular_z, max_linear=0.20, max_angular=0.45, mode=3):
    if not all(_finite(v) for v in (linear_x, angular_z)):
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


def service_frame(command):
    if command not in (SERVICE_ARM, SERVICE_DISARM):
        raise ValueError('unsupported service command')
    return b'\xbb\x55' + bytes([command, command])


def _finite(value):
    return value == value and abs(value) != float('inf')


def parse_telemetry(frame):
    """解析一帧 96 字节遥测；校验失败抛 ValueError。"""
    if len(frame) != TELEMETRY_SIZE:
        raise ValueError('bad telemetry length')
    if frame[0:2] != TELEMETRY_HEADER:
        raise ValueError('bad telemetry header')
    checksum = 0
    for byte in frame[2:-1]:
        checksum ^= byte
    if checksum != frame[-1]:
        raise ValueError('bad telemetry checksum')
    values = struct.unpack(_TELEMETRY_FORMAT, frame)
    return {
        'version': values[2],
        'flags': values[3],
        'mode': values[4],
        'seq': values[5],
        'tick_ms': values[6],
        'cmd_vx': values[7],
        'cmd_vz': values[8],
        'left_speed': values[9],
        'right_speed': values[10],
        'left_target': values[11],
        'right_target': values[12],
        'left_current': values[13],
        'right_current': values[14],
        'roll_deg': values[15],
        'pitch_deg': values[16],
        'yaw_deg': values[17],
        'gyro_deg_s': values[18:21],
        'accel_g': values[21:24],
        'req_left': values[24],
        'req_right': values[25],
        'last_result': values[26],
        'rx_frames': values[27],
        'rx_bad': values[28],
        'tx_busy': values[29],
    }


class TelemetryStream:
    """把 CDC 的字节流切成遥测帧，自动重同步与丢弃坏帧。"""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(TELEMETRY_HEADER)
            if start < 0:
                # 保留最后 1 字节，防止 0xCC 恰好落在本次读取末尾。
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < TELEMETRY_SIZE:
                break
            frame = bytes(self._buffer[:TELEMETRY_SIZE])
            try:
                frames.append(parse_telemetry(frame))
            except ValueError:
                del self._buffer[:2]
                continue
            del self._buffer[:TELEMETRY_SIZE]
        return frames
