"""STM32 -> Jetson 遥测帧的共用解析器。

为什么单独成模块：这份帧格式有两个地方极易搞错（见下面的 ⚠️），而这个仓库里
现在有两处消费者——`chassis_bridge.py`（把轮速/IMU 接进 robot_localization）
和 `tools/twist_sign_probe.py`（诊断探针）。解析逻辑只应该有一份，
否则改帧格式时必然漏改一处。

帧格式定义在固件 `Drivers/Hardware/bsp_usb.c: USB_SendSensorTelemetry()`
（结构体 `usb_sensor_telemetry_t` 在 `bsp_usb.h`），布局为
    CC 55 | version | flags | car_mode | ... | 载荷 | XOR 校验
校验和 = 字节 [2, size-1) 的异或；size 由帧内 version 决定：
    v1 = 151 字节（原始版）
    v2 = 183 字节（v1 的 151 字节【一个字节都没动】，只在后面追加了诊断段）
所以解析时必须【先按版本号选长度】，不能写死一个帧长。

⚠️ 两个已经踩过的坑，改动本文件前务必先读：

  1) `car_mode` 字段【不是】 CarMode_t，而是 JetsonMode_t。
     固件里 `telemetry.car_mode = chassis_telemetry_mode(chassis)`，
     那个函数把 CAR_MODE_* 映射成 JETSON_MODE_*：
         0=IDLE  1=GPS_ROS  2=REMOTE  3=INDOOR(即 CAR_MODE_ROS_INDOOR)  4=GPS_ONLY
     （曾经这里当成 CarMode_t 用，导致误报"固件没接受我们的帧"。）

  2) 各单位不是 SI，直接拿去发 ROS 消息会错：
     - gyro_* 单位是【°/s】，不是 rad/s（bsp_JY901S.c: raw/32768*2000 满量程 ±2000°/s）
       → 发 sensor_msgs/Imu 前要乘 DEG2RAD。
     - acc_*  单位是【g】，不是 m/s²（bsp_JY901S.c: raw/32768*16 满量程 ±16g）
       → 发 sensor_msgs/Imu 前要乘 G_TO_MS2。静止时 acc_z≈1.0，换算后≈9.8 才合理。
     - imu_roll/pitch/yaw 单位是【度】。
     - motor_speed[] 是【每 10 ms 控制周期的带符号整数编码器计数增量】，
       不是 rpm、也不是 m/s。满量程 ±120 counts（固件 ROS_LINE_MAX_SPEED）。

⚠️ 磁力计目前是坏的：`mag_yaw/mag_pitch/mag_roll` 恒为 0.00，
   而 `flags` 里的 MAG_VALID 位【照样置位】——所以那个标志位不可信，
   不要因为它有效就去用磁力计数据。
"""

import math
import struct

# ---------------------------------------------------------------------------
# 帧常量
# ---------------------------------------------------------------------------
FRAME_MAGIC = b'\xcc\x55'
# 版本 -> 帧长。新增字段时继续往后追加、并把版本号 +1，不要移动已有字段。
FRAME_SIZES = {1: 151, 2: 183}

# v1 字段偏移（帧内 0 基）。
OFF = {
    'version': 2, 'flags': 3, 'car_mode': 4, 'navigation_active': 9,
    'sequence': 12,
    'mag_yaw': 54, 'mag_pitch': 58, 'mag_roll': 62,
    'imu_roll': 66, 'imu_pitch': 70, 'imu_yaw': 74,
    'gyro_x': 78, 'gyro_y': 82, 'gyro_z': 86,
    'acc_x': 90, 'acc_y': 94, 'acc_z': 98,
    'motor_speed': 102,          # 4 个 float，连续 16 字节，顺序 FL FR RL RR
}
# v2 追加段偏移（紧接 v1 的 150 字节之后）：
#   150 u16 loop_period_ms   152 u16 busy_feedback   154 u16 busy_roadcls
#   156 u16 busy_control     158 u16 busy_send       160 4×i16 speed_set
#   168 4×i16 duty           176 f32 dist_counts     180 u16 tx_busy_count
#   182 校验和 -> 帧长 183
OFF_V2 = {
    'loop_period_ms': 150, 'busy_feedback_ms': 152, 'busy_roadcls_ms': 154,
    'busy_control_ms': 156, 'busy_send_ms': 158,
    'speed_set': 160, 'duty': 168,
    'dist_counts': 176, 'tx_busy_count': 180,
}

# 遥测 flags 位（bsp_usb.h）
FLAG_GPS_VALID = 0x01
FLAG_MAG_VALID = 0x02
FLAG_IMU_VALID = 0x04
FLAG_GNSS_HEAD_VALID = 0x08
FLAG_ROUTE_VALID = 0x10
FLAG_TARGET_VALID = 0x20

# ⚠️ 是 JetsonMode_t，不是 CarMode_t（见模块 docstring 的坑 1）
JETSON_MODES = {
    0: 'IDLE', 1: 'GPS_ROS', 2: 'REMOTE', 3: 'INDOOR(ROS)', 4: 'GPS_ONLY',
}
# 固件接受我们 mode=3 的帧之后应稳定在这一档
EXPECTED_TELEMETRY_MODE = 3

# 单位换算（见模块 docstring 的坑 2）
DEG2RAD = math.pi / 180.0
G_TO_MS2 = 9.80665

# 轮速里程计用到的常量
WHEEL_COUNT_PERIOD_S = 0.010          # 固件控制周期 10 ms（motor_speed 是"每周期"的计数）
WHEEL_FULL_SCALE_COUNTS = 120.0       # 固件 ROS_LINE_MAX_SPEED，仅命令尺度、物理未验证
WHEEL_TRACK_M = 0.2804                # 实测轮距（URDF 半轮距 0.1402 × 2）


def parse_frame(buf):
    """解析一帧遥测。按帧内版本号选长度；头部/版本/长度/校验任一不过即返回 None。

    返回的 dict 里数值一律保持【固件原始单位】（见 docstring 的坑 2），
    由调用方负责换算。v1 的帧不含 v2 的键。
    """
    if len(buf) < 3 or buf[:2] != FRAME_MAGIC:
        return None
    version = buf[OFF['version']]
    size = FRAME_SIZES.get(version)
    if size is None or len(buf) != size:
        return None
    checksum = 0
    for b in buf[2:size - 1]:
        checksum ^= b
    if checksum != buf[size - 1]:
        return None

    def f32(off):
        return struct.unpack_from('<f', buf, off)[0]

    def i16(off):
        return struct.unpack_from('<h', buf, off)[0]

    def u16(off):
        return struct.unpack_from('<H', buf, off)[0]

    out = {
        'version': version,
        'flags': buf[OFF['flags']],
        'car_mode': buf[OFF['car_mode']],
        'navigation_active': buf[OFF['navigation_active']],
        'sequence': u16(OFF['sequence']),
        'mag_yaw': f32(OFF['mag_yaw']),
        'mag_pitch': f32(OFF['mag_pitch']),
        'mag_roll': f32(OFF['mag_roll']),
        'imu_roll': f32(OFF['imu_roll']),
        'imu_pitch': f32(OFF['imu_pitch']),
        'imu_yaw': f32(OFF['imu_yaw']),
        'gyro_x': f32(OFF['gyro_x']),
        'gyro_y': f32(OFF['gyro_y']),
        'gyro_z': f32(OFF['gyro_z']),
        'acc_x': f32(OFF['acc_x']),
        'acc_y': f32(OFF['acc_y']),
        'acc_z': f32(OFF['acc_z']),
        'motor_speed': [f32(OFF['motor_speed'] + 4 * i) for i in range(4)],
    }

    if version >= 2:
        out.update({
            'loop_period_ms': u16(OFF_V2['loop_period_ms']),
            'busy_feedback_ms': u16(OFF_V2['busy_feedback_ms']),
            'busy_roadcls_ms': u16(OFF_V2['busy_roadcls_ms']),
            'busy_control_ms': u16(OFF_V2['busy_control_ms']),
            'busy_send_ms': u16(OFF_V2['busy_send_ms']),
            'speed_set': [i16(OFF_V2['speed_set'] + 2 * i) for i in range(4)],
            'duty': [i16(OFF_V2['duty'] + 2 * i) for i in range(4)],
            'dist_counts': f32(OFF_V2['dist_counts']),
            'tx_busy_count': u16(OFF_V2['tx_busy_count']),
        })
    return out


def wheel_twist_from_counts(counts, dt_s, counts_per_mps, track, rest_counts,
                           invert=False):
    """四轮计数 -> (vx, vy, wz, still)。纯函数（无 ROS 依赖），便于主机单测。

    counts: [FL, FR, RL, RR] 每控制周期的带符号计数增量（见本模块 docstring 的坑 2）
    dt_s:   实测控制周期（秒）。⚠️ 周期会变（实测 35~71 ms），速度必须除以它。
    counts_per_mps: 标定系数，"每米行程累加多少个每周期计数"（Σ c_mean），
                    ⚠️ 未标定，见 README 的直线标定步骤。
    rest_counts: 静息阈值；`|count| <= rest_counts` 视为静止。

    差速近似：左右两轮各取平均。纯前进时四轮同向、纯自转时左右两组反号，
    所以能把 vx 与 wz 分离，只丢掉横移自由度 —— 本车在 ROS 室内模式下从不横移
    （固件 Vy_set 恒 0、nav2 的 max_vel_y=0），故该近似对本车充分。
    实测原地左转时四轮为 [-26,+26,-26,+26]（FL,FR,RL,RR），与 REP-103 一致。

    ⚠️ still 时返回全 0 —— 这是 ZUPT（零速修正），EKF 靠它把静止时的速度钉在 0。
       注意给的是"0"，与标定系数无关，所以标定没做也能生效。
    ⚠️ wz 不可信：麦轮打滑使其高估（净打滑 ≈0.53），下游不要融合它。
    ⚠️ 符号约定：固件【前进 = 负的 Vx】，所以前进时 counts 是负的 —— 调用方
       必须传 invert=True（chassis_bridge 的 wheel_invert 默认为 True）。
       搞错会让发布的 vx 与指令反向，而 EKF 同时收到 rf2o(+)/轮速(-) 两路
       矛盾的平移证据、把前进量压掉（实测指令 4.69 m -> EKF 只报 0.32 m）。
    """
    c = [-x for x in counts] if invert else list(counts)
    cfl, cfr, crl, crr = c
    # ⚠️ 用 <= 而不是 <：编码器是整数计数、静止时会有 ±1 抖动，
    #    严格小于会把"±1 抖动"判成"在动"，ZUPT 就失效了。
    #    rest_counts=1.0 即"0 与 ±1 都算静止"（±1 count/周期 ≈ 2.4 mm/s，可忽略）。
    still = all(abs(x) <= rest_counts for x in c)
    if still:
        return 0.0, 0.0, 0.0, True
    if dt_s <= 0.0:
        dt_s = WHEEL_COUNT_PERIOD_S           # 周期字段还没填好时的兜底
    left = 0.5 * (cfl + crl)
    right = 0.5 * (cfr + crr)
    # ⚠️⚠️ 符号：本固件【前进时 motor_speed 为负】，所以【平移通道要取反】。
    #
    #   推导（不依赖真机数据，只用控制环必然成立的条件）：
    #     1) PID 的反馈约定要求 motor_speed 与 speed_set 同号 —— 否则就是正反馈，
    #        轮子会飞转；而实测没有，所以二者同号。
    #     2) ROS 前进 -> 桥接发 vx_frame = -v（固件约定"前进=负Vx"，
    #        见 chassis_protocol.speed_frame 的 `vx = -clamp(linear_x)`）
    #        -> 固件 Vx_set = vx_frame*120 < 0 -> 四轮 speed_set 全负
    #        -> 故【前进时四轮计数全为负】。
    #     3) 纯 Wz 时桥接再取反一次 -> motor[FL]=+Wz, motor[FR]=-Wz
    #        -> ROS 左转时 FL/RL 为负、FR/RR 为正
    #        -> 与探针实测的 [-26,+26,-26,+26]（当时确实是左转、陀螺确认为 CCW）吻合。
    #     结论：固件的 Vx 与 Wz 都相对 ROS 反了一次，于是
    #         均值(∝Vx) 反一次 -> 【错】,  差分(∝Wz) 反两次 -> 【对】。
    #       所以平移取反、旋转不取反。（曾经两者都不反，导致发布的 vx 与 DWB 的
    #       指令反向，EKF 同时收到 rf2o(+)/轮速(-) 两路矛盾的平移证据。）
    vx = -(0.5 * (left + right) / dt_s) / counts_per_mps
    vy = -(0.25 * (cfl - cfr + crl - crr) / dt_s) / counts_per_mps
    wz = ((right - left) / dt_s) / (counts_per_mps * track)
    return vx, vy, wz, False


class FrameAssembler:
    """串口字节流 -> 完整帧的重同步器。

    串口读到的是一段段字节，可能半帧开头、可能夹着噪声，所以必须：
    找 magic -> 用 magic 后面的版本号定帧长 -> 攒够整帧才交出 -> 否则留着等下一段。
    遇到未知版本号时只跳过 magic（2 字节）而不是整段丢弃，否则一个坏字节就能
    把后面所有好帧堵死。
    """

    def __init__(self):
        self._rx = bytearray()
        self.resync_bytes = 0      # 统计：被丢弃的字节数（诊断用）

    def feed(self, chunk):
        """喂入一段字节，返回本次能解出的【全部】帧（list[dict]）。"""
        self._rx.extend(chunk)
        frames = []
        while True:
            start = self._rx.find(FRAME_MAGIC)
            if start < 0:
                # 没有 magic：只留最后一个字节，防止 magic 被切成两半
                if len(self._rx) > 1:
                    self.resync_bytes += len(self._rx) - 1
                    del self._rx[:-1]
                break
            if start > 0:
                self.resync_bytes += start
                del self._rx[:start]
            if len(self._rx) < 3:
                break                      # 还不知道多长，等更多字节
            size = FRAME_SIZES.get(self._rx[2])
            if size is None:
                # 未知版本：只跳过 magic 继续找，别把后面堵死
                self.resync_bytes += 2
                del self._rx[:2]
                continue
            if len(self._rx) < size:
                break                      # 整帧还没到齐
            frame = bytes(self._rx[:size])
            del self._rx[:size]
            parsed = parse_frame(frame)
            if parsed is not None:
                frames.append(parsed)
            else:
                # 校验/头部不过：已经按 magic+长度切过，算丢一个整帧
                self.resync_bytes += size
        return frames
