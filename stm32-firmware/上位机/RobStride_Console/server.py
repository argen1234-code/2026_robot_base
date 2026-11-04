"""Local-only graphical motor console. All SWD I/O is serialized on one worker."""
import argparse
import csv
import io
import json
import math
import os
from pathlib import Path
import queue
import secrets
import sys
import threading
import time
import webbrowser
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from bridge import SwdBridge
from profile import build_profile
from bluetooth import BluetoothController

BASE = Path(getattr(sys, '_MEIPASS', Path(__file__).parent))
HOME = Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).parent
RESULTS = ['成功', '未知命令', '参数越界', '状态不允许', '确认键错误', '电机离线', 'CAN 发送失败',
           '参数读回不符/超时', '电机故障', '状态确认超时', 'CAN Bus-off', '保护触发',
           '保留结果 12', '健康检查失败', '速度超限', '服务间断', '正在执行', '固件忙']
# RS05 manual 261009: speed-mode 700A +/-33, 11 A physical maximum,
# acceleration table ceiling 150. Do not use RS01 or feedback encoding limits.
LIMITS = dict(target_rad_s=33.0, speed_limit_rad_s=33.0, current_limit_a=11.0, accel_rad_s2=150.0)
DEFAULTS = dict(target_rad_s=0.0, speed_limit_rad_s=33.0, current_limit_a=2.4, accel_rad_s2=20.0)
# Both firmware projects share the g_debug ABI. The AXF that matches the flash
# on the board is selected at connect time, newest build first. Never assume
# that the last built project is the one running.
AXF_CANDIDATES = (('RobStride05_H743', 'RobStride05_H743.axf'),
                  ('RobStride05_H743_HAL', 'RobStride05_H743_HAL.axf'))


def discover_axf_paths():
    found = []
    for base in list(HOME.parents)[:3]:
        for project, name in AXF_CANDIDATES:
            path = base / project / 'MDK-ARM' / 'Objects' / name
            if path.exists() and path not in found:
                found.append(path)
    return sorted(found, key=lambda p: p.stat().st_mtime, reverse=True)


def validate(command, payload):
    if type(command) is not int or command not in range(1, 7):
        raise ValueError('无效命令')
    if command in (4, 5, 6):
        return dict(DEFAULTS)
    values = {}
    for k, default in DEFAULTS.items():
        raw = payload.get(k, default)
        if type(raw) not in (float, int) or not math.isfinite(raw):
            raise ValueError('请输入有效有限数值')
        if abs(raw) > LIMITS[k]:
            raise ValueError(f'{k} 超过 RS05 参数范围 ±{LIMITS[k]:g}')
        # Firmware compares float32 bit-for-bit with the prepared limits.
        import struct
        values[k] = struct.unpack('<f', struct.pack('<f', raw))[0]
    for k in ('speed_limit_rad_s', 'current_limit_a', 'accel_rad_s2'):
        if not 0 < values[k] <= LIMITS[k]:
            raise ValueError(f'{k} 必须在 (0, {LIMITS[k]:g}] 内')
    if abs(values['target_rad_s']) > values['speed_limit_rad_s']:
        raise ValueError('目标速度超过速度上限')
    if command in (1, 2) and values['target_rad_s'] != 0:
        raise ValueError('准备和使能的目标必须为零')
    return values


def validate_led(payload):
    bounds={'enabled':(0,1),'mode':(0,7),'brightness':(0,255),'speed_percent':(1,400),
            'color_order':(0,1),'red':(0,255),'green':(0,255),'blue':(0,255),
            'cycle_interval_ms':(1000,600000)}
    if not isinstance(payload,dict) or set(payload)!=set(bounds)|{'cycle_modes'}:
        raise ValueError('请提供完整灯效配置')
    for name,(low,high) in bounds.items():
        if type(payload[name]) is not int or not low<=payload[name]<=high:
            raise ValueError(f'灯效参数 {name} 必须是 {low}–{high} 的整数')
    modes=payload['cycle_modes']
    if not isinstance(modes,list) or not 1<=len(modes)<=8 or any(type(m) is not int or not 1<=m<=7 for m in modes):
        raise ValueError('循环列表需包含 1–8 项灯效，每项为 1–7；支持重复项')
    return {**payload,'cycle_modes':list(modes)}


class Controller:
    def __init__(self, profiles):
        self.profiles = list(profiles) if isinstance(profiles, (list, tuple)) else [profiles]
        self.profile = self.profiles[0]
        self.bridge = None
        self.lock = threading.RLock()
        self.jobs = queue.PriorityQueue()
        self.serial = 0
        self.generation = 0
        self.connected = False
        self.snapshot = None
        self.last_sample = 0
        self.pending = None
        self.pending2 = None
        self.setup_pending = None
        self.startup = None
        self.startup_message = ''
        self.drive_owner = None
        self.drive_targets = (0.0,0.0)
        self.drive_client_at = self.drive_sent_at = 0.0
        self.logs = deque(maxlen=200)
        self.history = deque(maxlen=36000)
        self.chassis_history = deque(maxlen=18000)
        self.chassis_last_tick = None
        self.probe_list = []
        self.last_tick = None
        self.tick_changed = time.monotonic()
        self.running = True
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()
        self.log('就绪', '连接后自动完成双电机配置和零速使能，选择目标后即可运行')

    def log(self, kind, message):
        with self.lock:
            self.logs.append({'time': time.strftime('%H:%M:%S'), 'kind': kind, 'message': message})

    def enqueue(self, action, payload):
        with self.lock:
            self.serial += 1
            urgent = action in ('disconnect','stop_all','drive_exit') or action == 'command' and payload.get('command') in (4, 5, 6) or action=='chassis' and payload.get('command')==4
            if urgent:
                self.generation += 1  # Discard older queued movement after a disable.
            ticket = {'done': threading.Event(), 'result': None, 'error': None,
                      'action': action, 'payload': payload, 'generation': self.generation}
            self.jobs.put((0 if urgent else 1, self.serial, ticket))
            return ticket

    def status(self):
        with self.lock:
            m = (self.snapshot or {}).get('motor', {})
            running = bool(self.connected and m.get('state') == 3 and m.get('enabled') and m.get('enable_requested'))
            ready = self.connected and self.startup is None and all(
                (self.snapshot or {}).get(name,{}).get('enabled') and
                (self.snapshot or {}).get(name,{}).get('enable_requested') and
                (self.snapshot or {}).get(name,{}).get('mode')==2
                for name in ('motor','motor2'))
            return {'connected': self.connected, 'snapshot': self.snapshot,
                    'initializing': self.startup is not None, 'motors_ready': bool(ready),
                    'startup_message': self.startup_message,
                    'sample_age_ms': round((time.monotonic() - self.last_sample) * 1000) if self.last_sample else None,
                    'pending': self.pending, 'pending_motors': [self.pending,self.pending2],
                    'continuous': running, 'continuous_motors': [running,bool(self.connected and (self.snapshot or {}).get('motor2',{}).get('state')==3 and (self.snapshot or {}).get('motor2',{}).get('enable_requested') and (self.snapshot or {}).get('motor2',{}).get('enabled'))],
                    'motor_count': 2 if 'motor2' in self.profile['schema']['fields'] else 1,
                    'led_supported': 'led_command' in self.profile,
                    'drive_supported': 'drive_cmd' in self.profile['schema']['fields'],
                    'drive_active': self.drive_owner is not None,
                    'chassis_supported': 'chassis_cmd' in self.profile['schema']['fields'],
                    'setup_pending': self.setup_pending, 'logs': list(self.logs), 'probes': self.probe_list,
                    'profile': {'axf_sha256': self.profile['axf_sha256'],
                                'address': hex(self.profile['address']), 'source': self.profile['source']},
                    'history_count': len(self.history), 'limits': LIMITS, 'defaults': DEFAULTS}

    def loop(self):
        while self.running:
            try:
                _, _, ticket = self.jobs.get(timeout=0.025)
            except queue.Empty:
                ticket = None
            if ticket:
                try:
                    if ticket['generation'] != self.generation:
                        raise ValueError('已取消：后续失能或断开覆盖了此操作')
                    ticket['result'] = self.perform(ticket['action'], ticket['payload'])
                except Exception as exc:
                    ticket['error'] = str(exc)
                    self.log('拒绝', str(exc))
                finally:
                    ticket['done'].set()
            if self.connected:
                try:
                    self.service_drive()
                    snap = self.bridge.read()
                    self.service_startup(snap)
                    if self.drive_owner and not snap.get('drive',{}).get('active'):
                        self.drive_owner=None
                        self.log('小车模式停止','H7 已撤销小车模式，请检查反馈后重新进入')
                    now = time.monotonic()
                    tick = snap['system']['tick_ms']
                    if tick != self.last_tick:
                        self.tick_changed = now
                        self.last_tick = tick
                    if now - self.tick_changed > 0.5:
                        raise RuntimeError('H743 心跳停止，停止发布有效状态')
                    if snap['system']['fatal'] or snap['system']['boot_stage'] != 9:
                        raise RuntimeError('H743 系统异常，请检查 fatal 和启动阶段')
                    with self.lock:
                        # Each motor has its own publication sequence and receipt.
                        previous=self.snapshot or {}
                        merged=dict(snap)
                        for slot,name in enumerate(('motor','motor2')):
                            if name not in snap:
                                continue
                            valid=snap['coherent' if slot==0 else 'coherent2']
                            m=snap[name]
                            if valid:
                                self.history.append({'time':time.time(),'tick_ms':tick,'motor':slot+1,
                                    'motor_id':m.get('configured_id',127 if slot==0 else 126),'online':m['online'],
                                    **{k:m[k] for k in ('target_rad_s','applied_rad_s','velocity_rad_s',
                                    'position_rad','torque_nm','temperature_c','current_a','bus_voltage_v',
                                    'current_age_ms','voltage_age_ms','state')}})
                            else:
                                # Keep a last value but mark THIS channel invalid.
                                merged[name]=previous.get(name,m)
                            pending=self.pending if slot==0 else self.pending2
                            if pending and valid:
                                if m['executed_seq']==pending['seq']:
                                    code=m['result']
                                    self.log('完成' if code==0 else '拒绝',
                                        f"电机 {slot+1} · 序号 {pending['seq']} · {RESULTS[code] if code<len(RESULTS) else code}")
                                    if code==7:
                                        self.log('参数诊断',f"电机 {slot+1} · 索引 0x{m.get('readback_index',0):04X} · 状态 {m.get('readback_status',0)}")
                                    if slot==0:self.pending=None
                                    else:self.pending2=None
                                elif now-pending['at']>3:
                                    self.log('超时',f'电机 {slot+1} 命令未获固件确认；不会自动重试')
                                    if slot==0:self.pending=None
                                    else:self.pending2=None
                        self.snapshot=merged
                        chassis=snap.get('chassis',{})
                        if snap.get('chassis_coherent') and chassis.get('published_tick_ms')!=self.chassis_last_tick:
                            self.chassis_last_tick=chassis['published_tick_ms']
                            row={k:v for k,v in chassis.items() if isinstance(v,(int,float))}
                            row.update(time=time.time(),**{'imu_'+k:v for k,v in chassis['imu'].items()})
                            row.update({'config_'+k:v for k,v in chassis['config'].items()})
                            self.chassis_history.append(row)
                        self.last_sample=now
                        if self.setup_pending:
                            setup=snap.get('setup',{})
                            if setup.get('executed_seq')==self.setup_pending['seq']:
                                code=setup['result']
                                self.log('配置完成' if code==0 else '配置失败',
                                    f"{self.setup_pending['label']} · {RESULTS[code] if code<len(RESULTS) else code}")
                                self.setup_pending=None
                            elif now-self.setup_pending['at']>4:
                                self.log('配置超时','未获配置回执，请检查真实 ID，禁止直接重复修改')
                                self.setup_pending=None
                except Exception as exc:
                    self.log('连接中断', str(exc))
                    self.drop()

    def drop(self):
        with self.lock:
            self.startup = None
            self.startup_message = ''
            self.drive_owner = None
            self.connected = False
            self.pending = self.pending2 = self.setup_pending = None
        if self.bridge:
            try:
                self.bridge.close()
            except Exception as exc:
                self.log('关闭', str(exc))
            self.bridge = None

    def start_motors(self, parameters, generation):
        self.bridge.stop_all()
        self.startup = {'phase':'stopping','at':time.monotonic(),
                        'generation':generation,'params':parameters}
        self.startup_message = '正在自动配置双电机…'

    def service_startup(self, snap):
        with self.lock:
            self._service_startup(snap)

    def _service_startup(self, snap):
        job = self.startup
        if job is None:return
        if job['generation']!=self.generation:
            self.startup=None
            self.startup_message='自动就绪已取消；重新连接可恢复'
            return
        try:
            if time.monotonic()-job['at']>5:
                raise RuntimeError('等待双电机在线或操作回执超时')
            names=('motor','motor2')
            if not snap.get('coherent') or not snap.get('coherent2'):return
            motors=[snap[name] for name in names]
            def phase(name, message):
                job['phase']=name;job['at']=time.monotonic()
                self.startup_message=message
            def submit_pair(command):
                receipts=[]
                for slot,params in enumerate(job['params']):
                    seq=self.bridge.submit(command,params,motor=slot)
                    receipt={'seq':seq,'command':command,'at':time.monotonic()}
                    receipts.append(receipt)
                    if slot==0:self.pending=receipt
                    else:self.pending2=receipt
                job['receipts']=receipts
            if job['phase']=='stopping':
                if any(m['enable_requested'] or m['job_command'] or m['state'] not in (0,5) for m in motors):return
                if not all(m['online'] and m['identity_valid'] for m in motors):return
                job['seq']=self.bridge.setup(1,False)
                phase('identity','正在自动识别双电机…')
            elif job['phase']=='identity':
                if snap['setup']['executed_seq']!=job['seq']:return
                if snap['setup']['result'] or not snap['setup']['dual_ready']:
                    raise RuntimeError('双电机身份识别失败')
                submit_pair(1);phase('prepare','正在自动准备速度模式…')
            elif job['phase'] in ('prepare','enable'):
                if any(m['executed_seq']!=r['seq'] for m,r in zip(motors,job['receipts'])):return
                for slot,m in enumerate(motors):
                    if m['result']:
                        raise RuntimeError(f"电机 {slot+1}：{RESULTS[m['result']]}")
                if job['phase']=='prepare':
                    submit_pair(2);phase('enable','正在自动零速使能双电机…')
                elif all(m['mode']==2 and m['enabled'] and m['enable_requested'] and
                         m['target_rad_s']==0 and not m['job_command'] for m in motors):
                    self.startup=None;self.startup_message='双电机已就绪，可直接运行'
                    self.log('连接就绪','ID 127 / 126 已自动配置并零速使能，可直接使用角度、轮速或 WASD')
        except Exception as exc:
            self.startup=None;self.startup_message='自动就绪失败：'+str(exc)
            self.bridge.stop_all()
            self.pending=self.pending2=None
            self.log('自动就绪失败',str(exc))

    def service_drive(self):
        if not self.drive_owner:return
        now=time.monotonic()
        if now-self.drive_client_at>=0.65:
            self.end_drive()
            self.log('小车模式停止','键盘页面心跳中断，左右轮目标归零')
        elif now-self.drive_sent_at>=0.15:
            self.bridge.drive(2,self.drive_targets)
            self.drive_sent_at=time.monotonic()

    def end_drive(self):
        active=self.drive_owner is not None
        self.drive_owner=None
        self.drive_targets=(0.0,0.0)
        if active:self.bridge.drive(3)

    def perform_drive(self, action, payload):
        if not self.connected:raise ValueError('请先连接设备')
        if 'drive_cmd' not in self.profile['schema']['fields']:
            raise ValueError('请先下载支持 WASD 的 H7 固件')
        if action=='drive_exit':
            # A stale tab cannot release a newer tab's control session.
            if self.drive_owner and payload.get('owner')!=self.drive_owner:
                raise ValueError('小车控制会话已失效')
            self.end_drive()
            return {'active':False}
        if action=='drive_enter':
            if self.drive_owner:raise ValueError('小车模式已被占用，请先退出')
            if self.pending or self.pending2 or self.setup_pending:
                raise ValueError('请等待原操作完成')
            snap=self.bridge.read()
            if not snap.get('coherent') or not snap.get('coherent2'):
                raise ValueError('双电机快照不一致，请稍后再试')
            for name in ('motor','motor2'):
                m=snap[name]
                if m['state'] not in (2,3,4) or not m['enabled'] or not m['enable_requested'] or m['job_command']:
                    raise ValueError('双电机尚未就绪，请等待自动配置；失能后重新连接即可恢复')
            result=self.bridge.drive(1)
            self.drive_owner=secrets.token_urlsafe(16)
            self.drive_targets=(0.0,0.0)
            self.drive_client_at=self.drive_sent_at=time.monotonic()
            self.log('小车模式','W/S 前后，A/D 原地转向；松键归零，Esc 全部失能')
            return {**result,'owner':self.drive_owner}
        if not self.drive_owner or payload.get('owner')!=self.drive_owner:
            raise ValueError('小车控制会话已失效，请重新进入模式')
        targets=payload.get('targets')
        if not isinstance(targets,list) or len(targets)!=2:
            raise ValueError('需同时提供两轮目标速度')
        snap=self.bridge.read()
        converted=[]
        for value,name in zip(targets,('motor','motor2')):
            if type(value) not in (float,int) or not math.isfinite(value):
                raise ValueError('请输入有效轮速')
            # Reuse exactly the existing parameter validator and prepared caps.
            m=snap[name]
            params=validate(3,{k:m[k] for k in ('speed_limit_rad_s','current_limit_a','accel_rad_s2')}|{'target_rad_s':value})
            converted.append(params['target_rad_s'])
        self.drive_targets=tuple(converted)
        result=self.bridge.drive(2,self.drive_targets)
        self.drive_client_at=self.drive_sent_at=time.monotonic()
        return result

    def perform_chassis(self, action, payload):
        if not self.connected:raise ValueError('请先连接设备')
        if 'chassis_cmd' not in self.profile['schema']['fields']:
            raise ValueError('请下载 JY901S 航向控制新固件')
        snap=self.bridge.read()
        if action in ('chassis_prepare','chassis_enable'):
            if self.drive_owner or snap.get('chassis',{}).get('active'):
                raise ValueError('请先停止当前底盘控制')
            if self.pending or self.pending2:raise ValueError('请等待双轮操作完成')
            if not snap.get('setup',{}).get('dual_ready'):raise ValueError('请先核对双电机 ID')
            if not snap.get('coherent') or not snap.get('coherent2'):raise ValueError('请等待有效双轮快照')
            command=1 if action=='chassis_prepare' else 2
            params=[]
            for name in ('motor','motor2'):
                m=snap[name]
                if command==1:
                    if m['state'] not in (0,1,5) or m['enable_requested']:
                        raise ValueError('请先全部失能，再准备双轮速度模式')
                    params.append(validate(1,payload))
                else:
                    if m['state']!=1 or m['mode']!=2:raise ValueError('请先准备双轮速度模式')
                    params.append({k:m[k] for k in DEFAULTS})
                    params[-1]['target_rad_s']=0.0
            for slot,p in enumerate(params):
                seq=self.bridge.submit(command,p,motor=slot)
                receipt={'seq':seq,'command':command,'at':time.monotonic()}
                if slot==0:self.pending=receipt
                else:self.pending2=receipt
            self.log('双轮速度模式','已提交双轮准备' if command==1 else '已提交双轮使能')
            return {'submitted':True}
        command=payload.get('command')
        if type(command) is not int or command not in (1,2,3,4,5):raise ValueError('无效底盘命令')
        if command==4 and self.drive_owner:self.end_drive()
        config=dict(snap['chassis']['config'])
        fields=self.profile['schema']['fields']['chassis_cmd']['fields']
        updates=payload.get('config',{})
        if not isinstance(updates,dict):raise ValueError('请输入 PID 参数')
        for name,field in fields.items():
            if name in ('seq','key','command'):continue
            value=updates.get(name,config[name])
            if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('请输入有效数值：'+name)
            if field['kind']=='uint' and (type(value) is not int or value not in (0,1)):raise ValueError('无效左右轮配置')
            config[name]=value
        result=self.bridge.chassis(command,config)
        self.log('底盘控制',{1:'参数已应用',2:'双轮速度已应用',3:'绝对角度目标已应用',4:'底盘已停止',5:'当前位置已设为零度'}[command])
        return result

    def perform(self, action, payload):
        if self.startup and (action in ('stop_all','drive_exit') or
                action=='command' and payload.get('command') in (4,5,6) or
                action=='chassis' and payload.get('command')==4):
            self.startup=None
            self.startup_message='自动就绪已取消；重新连接可恢复'
            self.pending=self.pending2=None
            seq=self.bridge.stop_all()
            self.log('全部失能','自动就绪已取消，两台电机已请求失能')
            return {'seq':seq,'active':False}
        if self.startup and action in ('command','chassis','chassis_prepare','chassis_enable','drive_enter','setup'):
            raise ValueError('双电机正在自动配置，请稍候')
        if action in ('chassis','chassis_prepare','chassis_enable'):
            return self.perform_chassis(action,payload)
        if action in ('drive_enter','drive_update','drive_exit'):
            try:return self.perform_drive(action,payload)
            except Exception:
                # Invalid/torn/expired updates must not leave the old vector.
                if action=='drive_update' and payload.get('owner')==self.drive_owner:
                    try:self.end_drive()
                    except Exception:pass  # H7 lease also expires independently.
                raise
        if action=='led':
            if not self.connected:raise ValueError('请先连接设备')
            config=validate_led(payload)
            result=self.bridge.set_led(config)
            self.log('灯效已应用',f"序号 {result['seq']} · {'循环播放' if config['mode']==0 else '固定灯效'} · 亮度 {config['brightness']}/255")
            return result
        if action == 'probes':
            if self.connected:
                return self.probe_list
            self.probe_list = SwdBridge.probes()
            return self.probe_list
        if action == 'connect':
            if self.connected:
                raise ValueError('已经连接，请先断开')
            generation=self.generation
            drafts=payload.get('motor_params',[{},{}])
            if not isinstance(drafts,list) or len(drafts)!=2:
                raise ValueError('请提供两台电机的运行限制')
            parameters=[validate(1,{**p,'target_rad_s':0.0}) for p in drafts]
            probe_id = payload.get('probe_id')
            if probe_id not in [p['id'] for p in SwdBridge.probes()]:
                raise ValueError('请选择当前已连接的调试器')
            self.bridge = SwdBridge(self.profiles)
            self.log('连接', '正在核对板上完整 Flash 与 AXF，请稍候')
            snap = self.bridge.connect(probe_id)
            self.profile = self.bridge.profile
            with self.lock:
                self.snapshot = snap
                self.last_sample = self.tick_changed = time.monotonic()
                self.connected = True
                self.last_tick = snap['system']['tick_ms']
            project = Path(self.profile['source']).parent.parent.parent.name
            self.log('已连接', f'Flash 一致 · {project} · 正在自动准备双电机')
            self.start_motors(parameters,generation)
            return {'connected': True,'initializing':True}
        if action == 'disconnect':
            self.drive_owner=None
            if self.connected:
                try:
                    seq = self.bridge.stop_all()
                    self.log('失能请求', f'断开前提交序号 {seq}；断开后无法确认机械状态')
                finally:
                    self.drop()
            else:
                self.drop()
            self.log('已断开', 'SWD 会话已释放')
            return {'connected': False}
        if action in ('setup','stop_all'):
            if not self.connected:
                raise ValueError('请先连接设备')
            if action=='stop_all':
                self.drive_owner=None
                self.startup_message='双电机已失能，重新连接可自动就绪'
                seq=self.bridge.stop_all()
                self.pending=self.pending2=None
                self.log('全部失能',f'两台失能请求 {seq} 已提交，请查看两台反馈确认')
                return {'seq':seq}
            command=payload.get('command')
            if type(command) is not int or command not in (1,2):
                raise ValueError('无效配置操作')
            if self.setup_pending or self.pending or self.pending2:
                raise ValueError('请等待上一操作完成')
            snap=self.bridge.read()
            if 'setup' not in snap:
                raise ValueError('请下载 RTOS 双电机固件')
            if snap['setup']['busy'] or not snap['coherent'] or not snap.get('coherent2'):
                raise ValueError('配置忙或快照无效，请稍后再试')
            for name in ('motor','motor2'):
                m=snap[name]
                if m['enable_requested'] or m['job_command'] or m['state'] not in (0,5):
                    raise ValueError('请先点击两台全部失能并等待状态确认')
            if command==2 and payload.get('isolated') is not True:
                raise ValueError('改 ID 前必须只接通待修改的那一台电机')
            if command==1 and payload.get('pair_confirmed') is not True:
                raise ValueError('请确认两台已分别设置为 127 和 126，并已接回总线')
            seq=self.bridge.setup(command,payload.get('isolated') is True)
            self.setup_pending={'seq':seq,'at':time.monotonic(),'label':'单电机 127 → 126' if command==2 else '双电机身份核对'}
            self.log('配置提交',self.setup_pending['label']+'，等待固件回执')
            return {'seq':seq}
        if action != 'command' or not self.connected:
            raise ValueError('请先连接设备')
        motor=payload.get('motor',0)
        if type(motor) is not int or motor not in (0,1) or (motor==1 and 'motor2' not in self.profile['schema']['fields']):
            raise ValueError('无效电机通道或固件不支持双电机')
        pending=self.pending if motor==0 else self.pending2
        if self.setup_pending and payload.get('command') not in (4,5,6):
            raise ValueError('配置进行中，请稍后再试')
        command = payload.get('command')
        if self.drive_owner:
            if command in (4,5,6):self.end_drive()
            else:raise ValueError('请先退出小车模式，再操作独立电机')
        params = validate(command, payload)
        if command not in (4, 5, 6):
            snap = self.bridge.read()
            for _ in range(2):
                if snap['coherent' if motor==0 else 'coherent2']:
                    break
                snap = self.bridge.read()
            m = snap['motor' if motor==0 else 'motor2']
            if 'setup' in snap and not snap['setup']['dual_ready']:
                raise ValueError('双电机身份尚未就绪，请重新连接')
            if not snap['coherent' if motor==0 else 'coherent2']:
                raise ValueError('本次反馈快照不一致，请重试；未发送命令')
            if pending or m['job_command']:
                raise ValueError('固件正在执行上一命令')
            allowed = {1: (0, 1, 5), 2: (1,), 3: (2, 3, 4)}
            if m['state'] not in allowed[command]:
                raise ValueError('电机尚未就绪；失能或清错后重新连接即可恢复')
            if command in (2, 3) and (not m['online'] or m['feedback_age_ms'] >= 150):
                raise ValueError('电机反馈已离线或过期')
            if m['health_fault'] or m['fault_bits'] or m['fault']:
                raise ValueError('存在系统或电机故障，请先排查')
            if command in (2, 3):
                for key in ('speed_limit_rad_s', 'current_limit_a', 'accel_rad_s2'):
                    if params[key] != m[key]:
                        raise ValueError('运行限制已修改，请重新连接以应用')
        if command == 4 and pending and pending['command'] != 3:
            raise ValueError('操作处理中，请使用立即失能')
        seq = self.bridge.submit(command, params) if motor==0 else self.bridge.submit(command, params, motor=1)
        if command in (5,6):
            self.startup_message='电机已失能或清错，重新连接可自动就绪'
        with self.lock:
            receipt={'seq':seq,'command':command,'at':time.monotonic()}
            if motor==0:self.pending=receipt
            else:self.pending2=receipt
        if command == 3:
            self.log('持续运行', f"速度 {params['target_rad_s']:.3f} rad/s 已下发；无需续发，直到停止、失能或故障")
        self.log('已提交', f"电机 {motor+1} · {['', '准备', '使能', '运行', '减速停止', '立即失能', '清错'][command]} · 序号 {seq}，等待固件确认")
        return {'seq': seq, 'message': '已提交，等待固件确认'}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def send_data(self, code, data, content_type='application/json; charset=utf-8'):
        raw = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if isinstance(data, (dict, list)) else data
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(raw)

    def valid_host(self):
        return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

    def do_GET(self):
        if not self.valid_host():
            return self.send_data(403, {'error': 'Invalid Host'})
        path = urlsplit(self.path).path
        if path == '/api/status':
            return self.send_data(200, self.server.controller.status())
        if path == '/api/session':
            return self.send_data(200, {'token': self.server.token})
        if path == '/api/bluetooth/status':
            return self.send_data(200, self.server.bluetooth.status())
        if path == '/api/chassis-export':
            with self.server.controller.lock:rows=list(self.server.controller.chassis_history)
            if not rows:return self.send_data(400,{'error':'尚无底盘数据'})
            out=io.StringIO();writer=csv.DictWriter(out,fieldnames=rows[0].keys())
            writer.writeheader();writer.writerows(rows)
            return self.send_data(200,('\ufeff'+out.getvalue()).encode('utf-8'),'text/csv; charset=utf-8')
        if path == '/api/export':
            with self.server.controller.lock:
                rows = list(self.server.controller.history)
            if not rows:
                return self.send_data(400, {'error': '尚无一致的真实数据可导出'})
            out = io.StringIO()
            writer = csv.DictWriter(out, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
            return self.send_data(200, ('\ufeff' + out.getvalue()).encode('utf-8'), 'text/csv; charset=utf-8')
        assets = {'/': ('index.html', 'text/html; charset=utf-8'), '/style.css': ('style.css', 'text/css; charset=utf-8'),
                  '/app.js': ('app.js', 'application/javascript; charset=utf-8'),
                  '/led.js': ('led.js', 'application/javascript; charset=utf-8'),
                  '/drive.js': ('drive.js', 'application/javascript; charset=utf-8'),
                  '/chassis.js': ('chassis.js', 'application/javascript; charset=utf-8'),
                  '/chassis.css': ('chassis.css', 'text/css; charset=utf-8'),
                  '/bluetooth.js': ('bluetooth.js', 'application/javascript; charset=utf-8'),
                  '/bluetooth.css': ('bluetooth.css', 'text/css; charset=utf-8'),
                  '/template-ui.js': ('template-ui.js', 'application/javascript; charset=utf-8'),
                  '/template.css': ('template.css', 'text/css; charset=utf-8'),
                  '/signal-theme.css': ('signal-theme.css', 'text/css; charset=utf-8'),
                  '/distortion-grid.js': ('distortion-grid.js', 'application/javascript; charset=utf-8'),
                  '/footer-distortion.js': ('footer-distortion.js', 'application/javascript; charset=utf-8'),
                  '/signal-distortion.js': ('signal-distortion.js', 'application/javascript; charset=utf-8'),
                  '/pixel-field.js': ('pixel-field.js', 'application/javascript; charset=utf-8')}
        if path not in assets:
            return self.send_data(404, {'error': 'Not found'})
        name, kind = assets[path]
        self.send_data(200, (BASE / 'web' / name).read_bytes(), kind)

    def do_POST(self):
        origins = (f'http://127.0.0.1:{self.server.server_port}', f'http://localhost:{self.server.server_port}')
        if not self.valid_host() or self.headers.get('Origin') not in origins or not secrets.compare_digest(
                self.headers.get('X-Console-Token', ''), self.server.token):
            return self.send_data(403, {'error': '请求来源或会话验证失败'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('请求大小无效')
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError('请求必须是对象')
            action = urlsplit(self.path).path.removeprefix('/api/')
            if action.startswith('bluetooth/'):
                result = self.server.bluetooth.call(action.split('/',1)[1], payload)
                return self.send_data(200, {'ok': True, 'data': result})
            if action not in ('probes', 'connect', 'disconnect', 'command', 'shutdown', 'setup', 'stop_all', 'led',
                              'drive_enter','drive_update','drive_exit','chassis','chassis_prepare','chassis_enable'):
                return self.send_data(404, {'error': 'Unknown action'})
            ticket = self.server.controller.enqueue('disconnect' if action == 'shutdown' else action, payload)
            if not ticket['done'].wait(35):
                return self.send_data(504, {'error': '操作仍在处理中，请查看日志；不要重复点击'})
            if ticket['error']:
                return self.send_data(400, {'error': ticket['error']})
            self.send_data(200, {'ok': True, 'data': ticket['result']})
            if action == 'shutdown':
                threading.Thread(target=self.server.shutdown, daemon=True).start()
        except (ValueError, TypeError, KeyError) as exc:
            self.send_data(400, {'error': str(exc)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--axf', type=Path)
    args = parser.parse_args()
    if args.axf:
        profiles = [build_profile(args.axf)]
    else:
        profiles = []
        for path in discover_axf_paths():
            try:
                profiles.append(build_profile(path))
            except Exception as exc:
                print(f'跳过不可用的 AXF {path}: {exc}', flush=True)
        if not profiles:
            profiles = [json.loads((BASE / 'firmware_profile.json').read_text('utf-8'))]
    profile = profiles[0]
    print('候选固件 AXF: ' + ', '.join(Path(p['source']).name for p in profiles), flush=True)
    controller = Controller(profiles)
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    except OSError as exc:
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{args.port}/api/status', timeout=1) as response:
                existing = json.load(response)
            # Any candidate firmware is acceptable: the running server already
            # matched one of them against Flash when it connected.
            if existing.get('profile', {}).get('axf_sha256') in {item['axf_sha256'] for item in profiles}:
                if not args.no_browser:
                    webbrowser.open(f'http://127.0.0.1:{args.port}')
                return
        except Exception:
            pass
        raise SystemExit(f'端口 {args.port} 已占用，请关闭旧窗口或指定 --port 8766。{exc}')
    server.controller = controller
    server.bluetooth = BluetoothController()
    server.token = secrets.token_urlsafe(32)
    url = f'http://127.0.0.1:{server.server_port}'
    print(f'RobStride Console: {url}\n关闭此进程会尝试失能并释放调试器。', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.bluetooth.close()
        ticket = controller.enqueue('disconnect', {})
        ticket['done'].wait(5)
        controller.running = False
        server.server_close()


if __name__ == '__main__':
    main()
