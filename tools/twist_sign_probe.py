#!/usr/bin/env python3
"""角速度符号 / 转速 实测探针（一次性诊断工具，不属于 robot_base_bringup 包）。

目的
----
原地转弯"响应慢 + 卡顿"的候选根因里，有一个必须实测才能排除的：
**从 ROS 的 angular.z 到车体实际转向，符号到底对不对、转速到底有多少。**
这个脚本把三路数据同时对拍：

  1. 指令   —— 我自己发的 ROS 系 angular.z（正 = 逆时针/左转，REP-103）
  2. 反馈   —— rf2o 激光里程计的 /odom yaw（Nav2 唯一的转角反馈来源）
  3. 真值   —— STM32 遥测里的 gyro_z / imu_yaw / mag_yaw 以及四个轮速

判读方法（脚本末尾会自动打表）
  * 指令符号 vs gyro_z 积分符号  → 符号链路对不对
  * 指令符号 vs rf2o Δyaw 符号   → rf2o 的转角反馈可不可信（关键！）
  * 指令大小 vs |gyro_z| 均值    → 实际角速率达到指令的百分之几
  * 四轮 motor_speed 的相对符号  → 原地转的机构是否正确（左右两对反向）

⚠️ 安全
  * 只发角速度、线速度恒为 0，所以不会前后冲出去。
  * 任何退出路径（正常结束 / Ctrl-C / 异常）都会补发零速帧并 flush。
  * 默认 0.30 rad/s（低于 0.45 的上限）、每段 3 秒。第一次跑别加大。

⚠️ 端口独占
  chassis_bridge 以 exclusive=True 打开 /dev/stm32，两者不能同时开。
  所以启动 ROS 栈时必须 motor_enable:=false，把串口让给本脚本：

      export ROS_DOMAIN_ID=42          # 与同机其它仿真隔离（见 memory）
      ros2 launch robot_base_bringup robot_base.launch.py \
          slam:=false visualization:=none          # 只要 rf2o + 雷达 + TF

      # 另一个终端
      python3 tools/twist_sign_probe.py --rate 0.30 --duration 3.0

  没有 ROS 也能跑（--no-odom），但那样就少了"rf2o 可不可信"这一路。
"""

import argparse
import math
import statistics
import struct
import sys
import threading
import time
from pathlib import Path

import serial

# 复用仓库里唯一的那份协议实现，保证与 chassis_bridge 发出的帧逐字节一致。
sys.path.insert(0, str(
    Path(__file__).resolve().parent.parent
    / 'robot_ros2_ws/src/robot_base_bringup/scripts'))
from chassis_protocol import speed_frame  # noqa: E402

# ---------------------------------------------------------------------------
# STM32 -> Jetson 遥测帧（Drivers/Hardware/bsp_usb.c: USB_SendSensorTelemetry）
# ---------------------------------------------------------------------------
FRAME_MAGIC = b'\xcc\x55'
# 版本 -> 帧长。v2 不动 v1 的任何字节，只在 151 之后【追加】诊断段，
# 所以同一个解析器必须按版本号选长度（见 bsp_usb.h 的 usb_sensor_telemetry_t）。
FRAME_SIZES = {1: 151, 2: 183}
# v1 字段偏移（帧内 0 基）。只列本脚本用得到的。
OFF = {
    'version': 2, 'flags': 3, 'car_mode': 4, 'navigation_active': 9,
    'sequence': 12,
    'mag_yaw': 54, 'imu_roll': 66, 'imu_pitch': 70, 'imu_yaw': 74,
    'gyro_x': 78, 'gyro_y': 82, 'gyro_z': 86,
    'motor_speed': 102,          # 4 个 float，连续 16 字节
}
# v2 追加段偏移（紧接 v1 的 150 字节之后）。布局：
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
FLAG_MAG_VALID = 0x02
FLAG_IMU_VALID = 0x04

# ⚠️ 遥测里的 car_mode 字段【不是】 CarMode_t，而是 JetsonMode_t：
#    app_chassis_board.c 里 telemetry.car_mode = chassis_telemetry_mode(chassis)，
#    那个函数把 CAR_MODE_* 映射成 JETSON_MODE_*。所以：
#        1=GPS_ROS  2=REMOTE  3=INDOOR(=CAR_MODE_ROS_INDOOR)  4=GPS_ONLY  0=IDLE
#    （曾经把这里当成 CarMode_t 用，导致误报"固件没接受帧"。）
JETSON_MODES = {
    0: 'IDLE', 1: 'GPS_ROS', 2: 'REMOTE', 3: 'INDOOR(ROS)', 4: 'GPS_ONLY',
}
# 期望值：固件接受我们 mode=3 帧之后应稳定在这一档
EXPECTED_TELEMETRY_MODE = 3

# JY901S 陀螺满量程 ±2000°/s（bsp_JY901S.c: gyro = raw/32768*2000），
# 所以遥测里的 gyro_z 单位是【°/s】，不是 rad/s。积分出来是【度】。
DEG2RAD = math.pi / 180.0


def parse_frame(buf):
    """解析一帧遥测。按帧内版本号选长度，头部/版本/校验任一不过就返回 None。"""
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
        'imu_yaw': f32(OFF['imu_yaw']),
        'gyro_z': f32(OFF['gyro_z']),
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


class OdomWatcher:
    """后台线程订阅 /odom，记录 (t, yaw)。ROS 不可用时静默降级。"""

    def __init__(self, topic):
        self.topic = topic
        self.samples = []            # [(monotonic, yaw_rad)]
        self.error = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        try:
            import rclpy
            from nav_msgs.msg import Odometry

            rclpy.init()
            node = rclpy.create_node('twist_sign_probe_odom')
            node.create_subscription(Odometry, self.topic, self._on_odom, 20)
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
            rclpy.shutdown()
        except Exception as exc:                      # noqa: BLE001
            self.error = f'{type(exc).__name__}: {exc}'

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.samples.append((time.monotonic(), yaw))

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3.0)

    def window(self, t0, t1):
        """取 [t0, t1] 内的样本，返回 (起点 yaw, 终点 yaw)。"""
        sel = [y for t, y in self.samples if t0 <= t <= t1]
        if len(sel) < 2:
            return None
        return sel[0], sel[-1]


def unwrap_delta(y0, y1):
    """atan2 差归一到 (-pi, pi] —— 本测试每段 <90°，不会真的绕圈。"""
    d = (y1 - y0 + math.pi) % (2.0 * math.pi) - math.pi
    return d


def mean(values):
    return sum(values) / len(values) if values else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/stm32')
    ap.add_argument('--rate', type=float, default=0.30,
                    help='测试用角速度幅值 rad/s（默认 0.30，不要超过 0.45）')
    ap.add_argument('--duration', type=float, default=3.0,
                    help='每个方向持续秒数（默认 3.0）')
    ap.add_argument('--settle', type=float, default=2.0,
                    help='基线/间隔的静止秒数（默认 2.0）')
    ap.add_argument('--period', type=float, default=0.05,
                    help='发帧周期秒（默认 0.05 = 20Hz，与 chassis_bridge 一致）')
    ap.add_argument('--odom-topic', default='/odom')
    ap.add_argument('--no-odom', action='store_true',
                    help='完全跳过 ROS，只对拍指令与固件陀螺')
    ap.add_argument('--yes', action='store_true', help='跳过启动前的确认停顿')
    args = ap.parse_args()

    if abs(args.rate) > 0.45:
        sys.exit('refuse: --rate 超过 chassis_bridge 的 max_angular=0.45，'
                 '会在 bridge 那层被静默限幅，测不出真值')

    odom = None
    if not args.no_odom:
        odom = OdomWatcher(args.odom_topic)
        odom.start()
        time.sleep(1.5)                     # 等 DDS 发现 + 首批样本
        if odom.error:
            print(f'[warn] /odom 订阅失败，退化为两路对拍: {odom.error}')
        elif not odom.samples:
            print(f'[warn] {args.odom_topic} 上还没有数据 —— rf2o/雷达没起来？'
                  ' 仍然继续，但"rf2o 可不可信"这一路会缺失')

    print(f'打开 {args.port} ...')
    try:
        dev = serial.Serial(args.port, baudrate=115200, timeout=0,
                            write_timeout=0.2, exclusive=True)
    except (serial.SerialException, OSError) as exc:
        if odom:
            odom.stop()
        sys.exit(f'无法打开 {args.port}: {exc}\n'
                 '（正在被 chassis_bridge 占用？请用 motor_enable:=false 启动）')

    # 计划： (标签, ROS 系 angular.z)。标签用 ASCII，保证表格列对齐。
    #   pos(+wz) = 逆时针/左转（ROS REP-103 正方向）  neg(-wz) = 顺时针/右转
    phases = [
        ('base', 0.0),
        ('pos(+wz)', +args.rate),
        ('gap', 0.0),
        ('neg(-wz)', -args.rate),
        ('stop', 0.0),
    ]
    durs = [args.settle, args.duration, args.settle, args.duration, args.settle]

    tele = []       # [(t, dict)]  遥测样本
    log = []        # [(t, wz_sent)]
    rx = bytearray()
    stop_flag = threading.Event()
    # [上次发帧时刻, 观察到的最长发帧间隔]。间隔 >0.5s 会让固件看门狗把车停掉。
    last_send = [0.0, 0.0]

    def pump():
        """持续收遥测并解帧。与发帧同线程即可（发帧只是每 50ms 一次写）。

        ⚠️ 不要在这里用 timeout=0 的紧循环空转：那会疯狂抢 GIL，把主线程的
        50ms 发帧节奏拖乱，而固件 500ms 看门狗一旦超时就会把车停掉（表现为
        telemetry 里 INDOOR(ROS) 与 IDLE 交替出现）。改用 in_waiting 判断，
        没数据就让出 CPU。
        """
        while not stop_flag.is_set():
            try:
                waiting = dev.in_waiting
                if not waiting:
                    time.sleep(0.002)
                    continue
                chunk = dev.read(waiting)
            except (serial.SerialException, OSError):
                return
            if chunk:
                rx.extend(chunk)
            # 按 CC 55 重新同步，逐帧解（帧长取决于帧内的版本号）
            while True:
                start = rx.find(FRAME_MAGIC)
                if start < 0:
                    del rx[:-1]              # 只留最后一个字节，防止半截 magic
                    break
                if len(rx) < start + 3:
                    if start:
                        del rx[:start]
                    break
                size = FRAME_SIZES.get(rx[start + 2])
                if size is None:
                    # 未知版本号：只跳过 magic 继续找，别让一个坏字节把整段堵死
                    del rx[:start + 2]
                    continue
                if len(rx) < start + size:
                    if start:
                        del rx[:start]
                    break
                frame = bytes(rx[start:start + size])
                del rx[:start + size]
                parsed = parse_frame(frame)
                if parsed:
                    tele.append((time.monotonic(), parsed))

    pump_thread = threading.Thread(target=pump, daemon=True)
    pump_thread.start()

    if not args.yes:
        print(f'\n将发送：静止 {args.settle}s → +{args.rate} rad/s {args.duration}s → '
              f'静止 {args.settle}s → -{args.rate} rad/s {args.duration}s → 停止')
        print('确认车周围已清空、可以驱动电机。按 Enter 开始，Ctrl-C 取消...')
        try:
            input()
        except KeyboardInterrupt:
            dev.close()
            if odom:
                odom.stop()
            print('\n已取消，未发送任何指令。')
            return

    try:
        for (label, wz_ros), dur in zip(phases, durs):
            print(f'  → {label}  ({dur:.1f}s)')
            t_start = time.monotonic()
            t_end = t_start + dur
            next_send = t_start
            while time.monotonic() < t_end:
                now = time.monotonic()
                if now >= next_send:
                    # ⚠️ 与 chassis_bridge.tick() 完全一致：先取反，再封帧。
                    #    固件 Remote_ROS_Update 对 wz 不做任何符号处理。
                    dev.write(speed_frame(0.0, -wz_ros, mode=3))
                    log.append((now, wz_ros))
                    if last_send[0] > 0.0:
                        last_send[1] = max(last_send[1], now - last_send[0])
                    last_send[0] = now
                    next_send = now + args.period
                time.sleep(0.005)
            window_end = time.monotonic()
            # 记录该段的指令均值（含过渡），分析时按窗口切
            log.append((window_end, None))

        # 主动多停一会儿，让车彻底停稳
        for _ in range(20):
            dev.write(speed_frame(0.0, 0.0, mode=3))
            time.sleep(0.05)
    except KeyboardInterrupt:
        print('\n[Ctrl-C] 立即停轮...')
    finally:
        stop_flag.set()
        try:
            for _ in range(10):                 # 补发零速，确保固件那侧收到
                dev.write(speed_frame(0.0, 0.0, mode=3))
                dev.flush()
                time.sleep(0.02)
        except (serial.SerialException, OSError):
            pass
        dev.close()
        pump_thread.join(timeout=1.0)
        if odom:
            odom.stop()

    # ------------------------------------------------------------------ 分析
    report(tele, log, phases, odom, max_write_gap=last_send[1])


def report(tele, log, phases, odom, max_write_gap=0.0):
    """对拍并打表。返回判决 dict（供 --selftest 断言，也便于将来回归）。

    tele  : [(monotonic, 遥测 dict)]
    log   : [(monotonic, wz_ros 或 None)]  —— 段边界用 None 标记
    phases: [(label, wz_ros)]
    odom  : OdomWatcher 或 None
    """
    print(f'\n遥测帧 {len(tele)} 个，/odom 样本 {len(odom.samples) if odom else 0} 个')
    print(f'最长发帧间隔 {max_write_gap * 1000:.0f} ms'
          f'（固件看门狗 500 ms，超过就会掉回 IDLE）')
    if len(tele) < 50:
        print('[warn] 遥测帧太少 —— 固件在跑吗？档位/在线状态见下方。')
    if tele:
        f = tele[0][1]
        modes_seen = {d['car_mode'] for _, d in tele}
        n = len(tele)
        duty = sum(1 for _, d in tele if d['car_mode'] == EXPECTED_TELEMETRY_MODE) / n
        mode_txt = ', '.join(
            f'{m}={JETSON_MODES.get(m, "?")} {sum(1 for _, d in tele if d["car_mode"] == m) / n:.0%}'
            for m in sorted(modes_seen))
        print(f'遥测档位占比: {mode_txt}')
        print(f'  首帧 flags=0x{f["flags"]:02x} '
              f'MAG_VALID={bool(f["flags"] & FLAG_MAG_VALID)} '
              f'IMU_VALID={bool(f["flags"] & FLAG_IMU_VALID)}')
        if not (f['flags'] & FLAG_IMU_VALID):
            print('  ⚠️ IMU 未标记有效 —— gyro_z/imu_yaw 不可信，先查 JY901S 接线再测。')
        # 固件只有处在 CAR_MODE_ROS_INDOOR 且 jetson 在线时才会用我们的 cmd_vel。
        if EXPECTED_TELEMETRY_MODE not in modes_seen:
            print(f'  ⚠️ 从未出现 INDOOR(ROS)(={EXPECTED_TELEMETRY_MODE}) —— '
                  '固件没接受我们的帧，本次测量无效。按现象排查：')
            print('     0=IDLE     : 帧没到（端口/波特率/校验）或 500ms 看门狗超时')
            print('     2=REMOTE   : 帧里 mode 字节不是 3')
            print('     另有 BT_IsPowerless() 会强制零输出，遥控器上的急停要松开')
        elif duty < 0.9:
            print(f'  ⚠️ INDOOR(ROS) 只占 {duty:.0%} 的时间，其余掉回 IDLE —— '
                  '说明发帧有 >500ms 的空档，固件看门狗把车停了，结果不可靠。')

    # ---- v2 诊断：下位机控制环周期 / 耗时分布 / 链路丢帧 ----
    # 这一段回答"车为什么慢"里最容易被忽略的一层：控制环到底跑多快。
    if tele and tele[0][1].get('version', 1) >= 2:
        n2 = len(tele)
        span = tele[-1][0] - tele[0][0]
        print('\n---- 下位机控制环实测（v2 遥测） ----')
        if n2 >= 2 and span > 0:
            seqs = [d['sequence'] for _, d in tele]
            sd = [((seqs[i] - seqs[i - 1]) & 0xFFFF) for i in range(1, n2)]
            jumped = sum(1 for x in sd if x > 1)
            print(f'  遥测到达 {n2 / span:.1f} 帧/s；sequence 跳变(>1) {jumped}/{len(sd)} 次'
                  f'  -> {"无丢帧，该速率即固件真实发送率" if jumped == 0 else "有丢帧"}')
        lp = [d['loop_period_ms'] for _, d in tele]
        med = statistics.median(lp)
        print(f'  固件自报 loop_period_ms: 中位 {med:.0f} ms  min {min(lp)}  max {max(lp)}'
              f'  => 控制环约 {1000.0 / med:.1f} Hz（osDelay(10) 期望 100 Hz）')
        segs = (('busy_feedback_ms', '传感器刷新+SD'),
                ('busy_roadcls_ms', '路面识别+语音'),
                ('busy_control_ms', '模式+运动学+PID'),
                ('busy_send_ms', '电机输出+遥测+蓝牙'))
        tot = 0.0
        for key, seg_name in segs:
            vals = [d[key] for _, d in tele]
            m = statistics.median(vals)
            tot += m
            print(f'    {seg_name:<16} 中位 {m:5.1f} ms   max {max(vals):5.0f}')
        print(f'    四段合计 {tot:.1f} + osDelay(10) = {tot + 10:.1f} ms'
              f'，实测周期 {med:.0f} ms')
        print(f'    => 差额 ≈ {med - tot - 10:.0f} ms 未被上述四段覆盖：多为被更高'
              f'优先级任务抢占（如 LVGL 刷屏），也含 1ms 量化的误差')
        # 只在"有目标速度"的采样上取统计量——否则大量静止采样会把中位数压成 0
        moving = [d for _, d in tele if any(abs(v) > 0 for v in d['speed_set'])]
        if moving:
            # ⚠️ 取【绝对值】的统计量：正反两段的目标互为相反数，直接取中位数会是 0。
            #    符号正确性由上面的"机构自检"和 phase 表负责，这里只看量级够不够。
            print(f'  四轮 目标/实际/duty 的【绝对量级】（只统计有目标速度的 '
                  f'{len(moving)} 个采样；actual 应追上 target，duty 长期贴 99 = 力矩到顶）')
            for i2, wheel_name in enumerate(('FL', 'FR', 'RL', 'RR')):
                tgt = statistics.median([abs(d['speed_set'][i2]) for d in moving])
                act = statistics.median([abs(d['motor_speed'][i2]) for d in moving])
                dty = statistics.median([abs(d['duty'][i2]) for d in moving])
                ratio = f'{act / tgt:.2f}' if tgt > 1e-9 else '  - '
                print(f'    {wheel_name}: |target| {tgt:7.1f}  |actual| {act:7.1f}  '
                      f'|duty| {dty:7.1f}  |actual|/|target| {ratio}')
        else:
            print('  四轮：本次全程没有目标速度（没转过），跳过 target/actual 对比')
        print(f'  CDC 忙丢帧累计 tx_busy_count = {tele[-1][1]["tx_busy_count"]}')
        dd = tele[-1][1]['dist_counts'] - tele[0][1]['dist_counts']
        print(f'  dist_counts 本段增量 {dd:+.1f}'
              f'（⚠️非标准单位，需直线跑已知距离实测标定成米）')
    elif tele:
        print('\n(遥测为 v1，无控制环诊断字段；刷入含 v2 遥测的固件后可见)')

    # 按指令窗口切出分析区间（None 收尾）
    seg_bounds = []
    t_seg_start = None
    for t, wz in log:
        if wz is None:
            seg_bounds.append((t_seg_start, t))
            t_seg_start = None
        elif t_seg_start is None:
            t_seg_start = t

    # gyro 零偏：取第一个段（baseline 静止）的均值。静止段的积分因此约等于 0。
    bias = 0.0
    if seg_bounds:
        b0, b1 = seg_bounds[0]
        base = [d['gyro_z'] for t, d in tele if b0 <= t <= b1]
        if len(base) >= 5:
            bias = mean(base)
    print(f'gyro_z 零偏（baseline 段均值）= {bias:+.4f} °/s')

    # 列宽用 ASCII，避免中文字宽把表格挤歪
    print('\n' + '=' * 116)
    print(f'{"phase":<12}{"cmd_wz":>8}{"gyro_int":>10}{"imu_d":>9}{"odom_d":>9}'
          f'{"odo_sign":>9}{"meas_wz":>9}{"ratio":>8}{"ss_ratio":>9}'
          f'{"wheels(FL+RL/FR+RR)":>21}{"mode3":>7}')
    print('  cmd_wz/meas_wz = rad/s（ROS 系）  gyro_int/imu_d/odom_d = 度')
    print('  ratio = 全段实测/指令； ss_ratio = 后半程(稳态)实测/指令')
    print('-' * 116)

    ms_means = {}          # label -> [4 个轮速均值]
    verdict = {
        'gyro_sign': None, 'odom_sign': None,
        'rate_ratio': float('nan'), 'rate_ratio_steady': float('nan'),
        'meas_wz_steady': float('nan'), 'wheel_pattern_ok': None,
        'telemetry_mode_seen': sorted({d['car_mode'] for _, d in tele}) if tele else [],
        'mode3_duty': duty if tele else 0.0,
        'max_write_gap_s': max_write_gap,
    }
    for (label, wz_ros), (t0, t1) in zip(phases, seg_bounds):
        seg = [(t, d) for t, d in tele if t0 <= t <= t1]
        if len(seg) < 5:
            print(f'{label:<12}  遥测样本不足，跳过')
            continue

        # gyro_z 梯形积分（°/s × s = 度），先扣零偏
        gyro_int_deg = 0.0
        for (ta, da), (tb, db) in zip(seg, seg[1:]):
            gyro_int_deg += 0.5 * ((da['gyro_z'] - bias) + (db['gyro_z'] - bias)) * (tb - ta)

        imu_d_deg = math.degrees(unwrap_delta(seg[0][1]['imu_yaw'] * DEG2RAD,
                                              seg[-1][1]['imu_yaw'] * DEG2RAD))

        odom_d_deg = float('nan')
        odom_sign = '—'
        if odom and odom.samples:
            win = odom.window(t0, t1)
            if win:
                odom_d_deg = math.degrees(unwrap_delta(*win))
                if abs(wz_ros) > 1e-9:
                    odom_sign = 'same' if odom_d_deg * wz_ros > 0 else 'OPPOSITE'

        elapsed = t1 - t0
        meas_wz = (gyro_int_deg * DEG2RAD / elapsed) if elapsed > 0 else float('nan')
        ratio = abs(meas_wz) / abs(wz_ros) if abs(wz_ros) > 1e-9 else float('nan')

        # 稳态速率：只看该段后半程，避开起步时 PID/斜率限制的爬升，
        # 这才是"全舵能转多快"的真值。
        half = seg[len(seg) // 2:]
        ss_deg, ss_t = 0.0, 0.0
        for (ta, da), (tb, db) in zip(half, half[1:]):
            ss_deg += 0.5 * ((da['gyro_z'] - bias) + (db['gyro_z'] - bias)) * (tb - ta)
            ss_t += (tb - ta)
        ss_wz = (ss_deg * DEG2RAD / ss_t) if ss_t > 0 else float('nan')
        ss_ratio = abs(ss_wz) / abs(wz_ros) if abs(wz_ros) > 1e-9 else float('nan')
        seen = [d['car_mode'] for _, d in seg]
        dom = max(set(seen), key=seen.count)
        mode3 = sum(1 for m in seen if m == EXPECTED_TELEMETRY_MODE) / len(seen)

        ms = [mean([d['motor_speed'][i] for _, d in seg]) for i in range(4)]
        ms_means[label] = ms
        # 左右两对的符号关系，一眼看出机构对不对
        left, right = ms[0] + ms[2], ms[1] + ms[3]

        print(f'{label:<12}{wz_ros:>8.3f}{gyro_int_deg:>10.2f}{imu_d_deg:>9.2f}'
              f'{odom_d_deg:>9.2f}{odom_sign:>9}{meas_wz:>9.3f}{ratio:>8.2f}'
              f'{ss_ratio:>9.2f}'
              f'{("L%+.1f R%+.1f" % (left, right)):>21}{mode3:>7.0%}'
              f'   [{JETSON_MODES.get(dom, "?")}]')

        # 只在"指令非零 + 固件确实在 INDOOR(ROS)"的段上做判决
        if abs(wz_ros) > 1e-9 and dom == EXPECTED_TELEMETRY_MODE:
            if verdict['gyro_sign'] is None:
                verdict['gyro_sign'] = 'same' if ss_wz * wz_ros > 0 else 'inverted'
                verdict['rate_ratio'] = ratio
                verdict['rate_ratio_steady'] = ss_ratio
                verdict['meas_wz_steady'] = ss_wz
            if verdict['odom_sign'] is None and odom_sign != '—':
                verdict['odom_sign'] = ('same' if odom_sign == 'same' else 'inverted')
            if verdict['wheel_pattern_ok'] is None:
                # 判据：左边一对(FL,RL)同号、右边一对(FR,RR)同号，且左右互为反号。
                # 只在指令非零的段上判，静止段的噪声不能用来下结论。
                verdict['wheel_pattern_ok'] = (left * right < 0)
                print(f'  机构自检 [{label}]: 左对={left:+.2f} 右对={right:+.2f} → '
                      f'{"左右反号 OK" if left * right < 0 else "左右同号 ✗ 原地转的轮向不对"}')

    print('\n四轮 motor_speed 均值  [0]=FL [1]=FR [2]=RL [3]=RR')
    for label, ms in ms_means.items():
        print(f'  {label:<12} ' + ' '.join(f'{v:>9.2f}' for v in ms))

    print('\n' + '=' * 104)
    print('判读提示：')
    print('  * gyro_int 与 cmd_wz 同号且 ratio≈1 → 符号链路正确且速率达标')
    print('  * gyro_int 与 cmd_wz 反号           → 符号反了（bridge 的取反过度/不足）')
    print('  * odom_d 与 cmd_wz 反号             → rf2o 的转角反馈不可信（Nav2 会发散）')
    print('  * 四轮全是 ~0 → 电机根本没转：先确认底盘电机电源/开关，再看 PWM 使能')
    print('  * ratio 只有百分之几十 → 执行链路（固件缩放/PID/麦轮打滑）的问题')
    return verdict


def selftest():
    """用合成数据跑通 report() 与全部判决逻辑，不需要硬件/ROS。"""

    # --- 帧解析自检：v1 与 v2 都要能解，坏校验/坏版本/错长度都要被拒 ---
    def build(version):
        size = FRAME_SIZES[version]
        buf = bytearray(size)
        buf[0], buf[1] = 0xCC, 0x55
        buf[OFF['version']] = version
        buf[OFF['flags']] = FLAG_MAG_VALID | FLAG_IMU_VALID
        struct.pack_into('<H', buf, OFF['sequence'], 1234)
        for off, val in ((OFF['mag_yaw'], 350.0), (OFF['imu_yaw'], 12.5),
                         (OFF['gyro_z'], -0.42)):
            struct.pack_into('<f', buf, off, val)
        for i in range(4):
            struct.pack_into('<f', buf, OFF['motor_speed'] + 4 * i,
                             [18.0, -18.0, 18.0, -18.0][i])
        if version >= 2:
            struct.pack_into('<H', buf, OFF_V2['loop_period_ms'], 71)
            struct.pack_into('<H', buf, OFF_V2['busy_feedback_ms'], 12)
            struct.pack_into('<H', buf, OFF_V2['busy_roadcls_ms'], 3)
            struct.pack_into('<H', buf, OFF_V2['busy_control_ms'], 1)
            struct.pack_into('<H', buf, OFF_V2['busy_send_ms'], 2)
            for i in range(4):
                struct.pack_into('<h', buf, OFF_V2['speed_set'] + 2 * i,
                                 [18, -18, 18, -18][i])
                struct.pack_into('<h', buf, OFF_V2['duty'] + 2 * i,
                                 [42, -42, 42, -42][i])
            struct.pack_into('<f', buf, OFF_V2['dist_counts'], 123.5)
            struct.pack_into('<H', buf, OFF_V2['tx_busy_count'], 7)
        ck = 0
        for b in buf[2:size - 1]:
            ck ^= b
        buf[size - 1] = ck
        return bytes(buf)

    for ver in (1, 2):
        frame = build(ver)
        r = parse_frame(frame)
        assert r and r['version'] == ver, (ver, r)
        assert r['sequence'] == 1234 and abs(r['gyro_z'] + 0.42) < 1e-6, r
        assert r['motor_speed'] == [18.0, -18.0, 18.0, -18.0], r
        if ver == 2:
            assert r['loop_period_ms'] == 71, r
            assert r['speed_set'] == [18, -18, 18, -18], r['speed_set']
            assert r['duty'] == [42, -42, 42, -42], r['duty']
            assert abs(r['dist_counts'] - 123.5) < 1e-3, r['dist_counts']
            assert r['tx_busy_count'] == 7, r
        else:
            assert 'loop_period_ms' not in r, 'v1 不该有 v2 字段'
        bad = bytearray(frame)
        bad[FRAME_SIZES[ver] - 1] ^= 0xFF
        assert parse_frame(bytes(bad)) is None, f'坏校验和应被拒 v{ver}'
    # 长度/版本不匹配必须被拒（避免把 v2 当 v1 切）
    assert parse_frame(build(2)[:151]) is None, 'v2 帧截成 151 不应通过'
    assert parse_frame(build(1) + b'\x00' * 32) is None, 'v1 帧补长不应通过'
    bad_ver = bytearray(build(1))
    bad_ver[OFF['version']] = 9
    assert parse_frame(bytes(bad_ver)) is None, '未知版本应被拒'
    print('帧解析/校验 OK（v1=151 与 v2=183 各自可解，坏校验/坏版本/错长度均被拒）')

    # --- 合成一次"符号正确、速率只有指令 83%"的测量 ---
    T0 = 1000.0
    settle, dur, rate_cmd = 2.0, 3.0, 0.30
    bias_dps, true_rate = 0.012, 0.25      # 陀螺零偏(°/s) / 实际角速率(rad/s)
    true_rate_dps = math.degrees(true_rate)   # 遥测里 gyro_z 的单位是 °/s
    plan = [('base', 0.0, settle), ('pos(+wz)', +rate_cmd, dur),
            ('gap', 0.0, settle), ('neg(-wz)', -rate_cmd, dur),
            ('stop', 0.0, settle)]

    tele, log = [], []
    t = T0
    yaw_imu, yaw_odom = 100.0, 0.0
    seq = 0
    # 正转段 gyro_z>0（CCW+）；imu_yaw 是罗盘约定 CW+，故反向走
    for label, wz, dur_s in plan:
        log.append((t, None if wz is None else wz))
        t_start = t
        rate_dps = (true_rate_dps if wz > 0 else
                    (-true_rate_dps if wz < 0 else 0.0))
        while t < t_start + dur_s:
            yaw_imu -= rate_dps * 0.01
            yaw_odom += rate_dps * DEG2RAD * 0.01
            seq += 1
            # 带上 v2 的字段，好让 report() 里的 v2 诊断段真的被执行到
            tele.append((t, {
                'version': 2, 'flags': FLAG_MAG_VALID | FLAG_IMU_VALID,
                'car_mode': EXPECTED_TELEMETRY_MODE, 'navigation_active': 1,
                'sequence': seq,
                'mag_yaw': (yaw_imu + 5.0) % 360.0,
                'imu_yaw': yaw_imu % 360.0, 'gyro_z': bias_dps + rate_dps,
                'motor_speed': ([18.0, -18.0, 18.0, -18.0] if wz > 0
                                else ([-18.0, 18.0, -18.0, 18.0] if wz < 0
                                      else [0.0, 0.0, 0.0, 0.0])),
                'loop_period_ms': 71, 'busy_feedback_ms': 12,
                'busy_roadcls_ms': 3, 'busy_control_ms': 1, 'busy_send_ms': 2,
                'speed_set': ([18, -18, 18, -18] if wz > 0
                              else ([-18, 18, -18, 18] if wz < 0
                                    else [0, 0, 0, 0])),
                'duty': ([42, -42, 42, -42] if wz > 0
                         else ([-42, 42, -42, 42] if wz < 0
                               else [0, 0, 0, 0])),
                'dist_counts': yaw_odom * 10.0,
                'tx_busy_count': 0,
            }))
            t += 0.01
        log.append((t, None))

    class FakeOdom:
        samples = []
    fake = FakeOdom()
    t, yaw = T0, 0.0
    for label, wz, dur_s in plan:
        rate = true_rate if wz > 0 else (-true_rate if wz < 0 else 0.0)
        end = t + dur_s
        while t < end:
            fake.samples.append((t, yaw))
            yaw += rate * 0.01
            t += 0.01
    fake.samples.append((t, yaw))
    fake.window = OdomWatcher.window.__get__(fake)

    phases = [(label, wz) for label, wz, _ in plan]
    v = report(tele, log, phases, fake)
    assert v['gyro_sign'] == 'same', v
    assert v['odom_sign'] == 'same', v
    assert abs(v['rate_ratio'] - true_rate / rate_cmd) < 0.05, v
    assert abs(v['rate_ratio_steady'] - true_rate / rate_cmd) < 0.05, v
    assert v['wheel_pattern_ok'] is True, v
    assert v['telemetry_mode_seen'] == [EXPECTED_TELEMETRY_MODE], v
    assert v['mode3_duty'] > 0.99, v
    print(f'\nselftest 全部通过: {v}')

    # --- 反例：故意把 gyro 符号翻过来，必须被判成 inverted ---
    for _, d in tele:
        d['gyro_z'] = -d['gyro_z']
    v2 = report(tele, log, phases, fake)
    assert v2['gyro_sign'] == 'inverted', v2
    print(f'selftest 反例(符号翻转)正确识别: gyro_sign={v2["gyro_sign"]}')
    print('\nselftest OK —— 分析路径与判决逻辑可用。')


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        selftest()
    else:
        main()
