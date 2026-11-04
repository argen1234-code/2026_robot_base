"""JDY31 SPP transport. One worker owns the serial port and renews live input."""
from collections import deque
import math
import queue
import secrets
import threading
import time

import serial
from serial.tools import list_ports

ERRORS = {1:'未知指令',2:'参数超出范围',3:'请先使能蓝牙遥控',4:'双电机身份未确认',
          5:'电机离线或串口接收错误',6:'CAN 发送失败',7:'参数读回失败',8:'电机故障',
          9:'遥控续发超时，已停车，请重新使能',10:'CAN Bus-off',11:'电机保护',
          13:'健康检查失败',14:'超速',15:'电机任务间断',17:'其他控制模式占用或正在运动'}
DIRECTIONS = {'F','B','L','R','Q','E','FL','FR','BL','BR','S'}


class BluetoothController:
    def __init__(self, serial_factory=None, clock=time.monotonic):
        self.factory = serial_factory or serial.Serial
        self.clock = clock
        self.jobs = queue.Queue()
        self.lock = threading.RLock()
        self.port = None
        self.running = True
        self.session = None
        self.direction = 'S'
        self.lease = self.sent_at = self.polled_at = 0.0
        self.rx = bytearray()
        self.lines = deque(maxlen=200)
        self.line_number = 0
        self.input_seq = -1
        self.state = dict(connected=False, port='', identity='', armed=False, phase=0,
                          moving=False, speed=1.0, result=0, error='', last_rx_at=None)
        self.thread = threading.Thread(target=self._worker, name='JDY31', daemon=True)
        self.thread.start()

    def status(self):
        with self.lock:
            age = None if self.state['last_rx_at'] is None else self.clock()-self.state['last_rx_at']
            return {**self.state, 'rx_age_s': age, 'direction': self.direction,
                    'logs':[{'time':t,'text':line} for _,t,line in self.lines][-35:]}

    def call(self, action, payload=None):
        ticket = dict(action=action, payload=payload or {}, done=threading.Event(), error=None, result=None)
        self.jobs.put(ticket)
        if not ticket['done'].wait(8):
            raise ValueError('蓝牙串口操作超时，请检查连接')
        if ticket['error']:
            raise ValueError(ticket['error'])
        return ticket['result']

    def close(self):
        try:
            self.call('disconnect')
        finally:
            self.running = False
            self.thread.join(2)

    def _log(self, line):
        with self.lock:
            self.line_number += 1
            self.lines.append((self.line_number,time.strftime('%H:%M:%S'),line))

    def _write(self, payload):
        if not self.port:
            raise ValueError('请先连接 Windows 配对后生成的蓝牙串口')
        raw = ('@'+payload+'\r\n').encode('ascii')
        if self.port.write(raw) != len(raw):
            raise OSError('蓝牙串口未完整发送指令')
        self._log('TX @'+payload)

    def _receive(self):
        raw = self.port.read(min(self.port.in_waiting or 1, 1024))
        self.rx.extend(raw)
        if len(self.rx)>4096:
            self.rx.clear()
            raise OSError('蓝牙返回数据没有有效换行')
        while b'\n' in self.rx:
            raw,_,rest = self.rx.partition(b'\n')
            self.rx[:] = rest
            line = raw.rstrip(b'\r').decode('ascii',errors='replace')
            self._log('RX '+line)
            with self.lock:
                self.state['last_rx_at'] = self.clock()
                if line.startswith('PONG '):
                    self.state['identity'] = line[5:]
                elif line.startswith('STATE '):
                    values = dict(part.split('=',1) for part in line.split()[1:] if '=' in part)
                    for key in ('armed','moving'):
                        if key in values:self.state[key] = values[key]=='1'
                    for key in ('phase','result'):
                        if key in values:self.state[key] = int(values[key])
                    for key in ('speed','v1','v2'):
                        if key in values:
                            value=float(values[key])
                            if not math.isfinite(value):raise OSError('蓝牙状态包含无效数值')
                            self.state[key] = value
                    if not self.state['armed']:self.direction='S'
                elif line == 'OK ARM':
                    self.state.update(armed=True,phase=0,result=0,error='')
                elif line == 'PENDING ARM':
                    self.state.update(armed=False,phase=1,result=16)
                elif line == 'OK DISARM':
                    self.state.update(armed=False,phase=0,moving=False,result=0,error='')
                    self.direction='S'
                elif line == 'OK S':
                    self.state.update(moving=False,result=0,error='')
                elif line.startswith('OK '):
                    self.state.update(result=0,error='')
                elif line.startswith('ERR '):
                    code=int(line[4:])
                    self.state.update(result=code,error=ERRORS.get(code,f'固件拒绝：{code}'))
                    if code in (5,8,9,10,11,13,14,15):
                        self.direction='S'
                        self.state.update(armed=False,moving=False,phase=0)

    def _exchange(self, payload, expected):
        # Drain old traffic before associating a fresh command with its receipt.
        if self.port.in_waiting:self._receive()
        start = self.line_number
        self._write(payload)
        deadline = self.clock()+1.8
        while self.clock()<deadline:
            self._receive()
            for number,_,line in list(self.lines):
                if number<=start or not line.startswith('RX '):continue
                receipt=line[3:]
                if receipt.startswith('ERR '):
                    code=int(receipt[4:])
                    raise ValueError(ERRORS.get(code,f'固件拒绝：{code}'))
                if receipt in expected:return receipt
        raise OSError('串口已发送，但未收到新版 H7 固件的有效回执')

    @staticmethod
    def _number(payload, key, minimum, maximum):
        value=payload.get(key)
        if type(value) not in (int,float) or not math.isfinite(value) or not minimum<=value<=maximum:
            raise ValueError(f'{key} 必须在 {minimum}–{maximum} 之间')
        return value

    def _perform(self, action, p):
        if action=='ports':
            return [dict(device=x.device,description=x.description) for x in list_ports.comports()]
        if action=='disconnect':
            self._disconnect()
            return {}
        if action=='connect':
            name=p.get('port')
            if not isinstance(name,str) or not name or len(name)>128:raise ValueError('请选择蓝牙串口')
            self._disconnect()
            try:
                self.port=self.factory(port=name,baudrate=9600,timeout=.015,write_timeout=.2)
                self.port.reset_input_buffer()
                self.rx.clear()
                self._exchange('PING',{'PONG RS05-BT/1'})
                self.session=secrets.token_urlsafe(16)
                self.input_seq=-1
                self.state.update(connected=True,port=name,error='')
                self._write('STATUS')
                return {'session':self.session}
            except Exception:
                self._disconnect()
                raise
        if not self.port or not self.state['connected']:raise ValueError('请先连接蓝牙串口')
        if action=='stop':
            self.direction='S'
            self._exchange('!' if p.get('disable') else 'S',{'OK DISARM'} if p.get('disable') else {'OK S'})
            return {}
        if p.get('session')!=self.session:raise ValueError('蓝牙连接已变化，请重新连接此页面')
        if action=='arm':
            self.direction='S'
            self._exchange('S',{'OK S'})
            self._exchange('!',{'OK DISARM'})
            cap=self._number(p,'cap',.001,33)
            current=self._number(p,'current',.001,11)
            accel=self._number(p,'accel',.001,150)
            speed=self._number(p,'speed',0,33)
            if speed>cap:raise ValueError('行驶速度不能超过准备时的速度上限')
            slot=p.get('left_slot');left=p.get('left_sign');right=p.get('right_sign')
            if type(slot) is not int or slot not in (0,1) or type(left) is not int or type(right) is not int or left not in (-1,1) or right not in (-1,1):
                raise ValueError('轮子安装方向无效')
            self._exchange(f'LIMIT={cap:g},{current:g},{accel:g}',{'OK LIMIT'})
            self._exchange(f'MAP={slot},{left},{right}',{'OK MAP'})
            self._exchange(f'SPEED={speed:g}',{'OK SPEED'})
            self._exchange('ARM',{'PENDING ARM','OK ARM'})
            return {}
        if action=='motion':
            direction=p.get('direction');seq=p.get('seq')
            if direction not in DIRECTIONS or type(seq) is not int:raise ValueError('方向指令无效')
            if seq<=self.input_seq:return {'stale':True}
            self.input_seq=seq
            if direction!='S' and not self.state['armed']:raise ValueError('请先使能蓝牙遥控')
            changed=direction!=self.direction
            self.direction=direction;self.lease=self.clock()
            if changed or direction=='S':
                self._exchange(direction,{'OK S'} if direction=='S' else {'OK MOVE'})
                self.sent_at=self.clock()
            return {}
        if action=='speed':
            speed=self._number(p,'speed',0,33)
            self.direction='S'
            self._exchange('S',{'OK S'})
            self._exchange(f'SPEED={speed:g}',{'OK SPEED'})
            return {}
        if action=='query':
            self._write('STATUS')
            return {}
        raise ValueError('未知蓝牙操作')

    def _service(self):
        now=self.clock()
        last_rx=self.state['last_rx_at']
        if last_rx is not None and now-last_rx>2.5:
            raise OSError('蓝牙固件回执中断，已撤销遥控连接')
        if self.direction!='S':
            if now-self.lease>=.65:
                self.direction='S';self._write('S')
            elif now-self.sent_at>=.18:
                self._write(self.direction);self.sent_at=now
        if now-self.polled_at>=.5:
            self._write('STATUS');self.polled_at=now
        self._receive()

    def _disconnect(self):
        self.direction='S'
        if self.port:
            try:
                if self.state['connected']:self._exchange('!',{'OK DISARM'})
            except Exception:pass
            try:self.port.close()
            finally:self.port=None
        with self.lock:
            self.session=None
            self.state.update(connected=False,armed=False,moving=False,phase=0)

    def _worker(self):
        while self.running:
            try:ticket=self.jobs.get(timeout=.01)
            except queue.Empty:ticket=None
            if ticket:
                try:ticket['result']=self._perform(ticket['action'],ticket['payload'])
                except Exception as exc:
                    ticket['error']=str(exc)
                    self.state['error']=str(exc)
                    if ticket['action']=='motion':self.direction='S'
                    if isinstance(exc,(OSError,serial.SerialException)):
                        self._disconnect()
                finally:ticket['done'].set()
            if self.port and self.state['connected']:
                try:self._service()
                except Exception as exc:
                    self.state['error']=str(exc);self._log('ERROR '+str(exc));self._disconnect()
