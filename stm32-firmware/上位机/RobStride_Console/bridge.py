"""Real running-target access only; no halt, reset, flash or auto-enable."""
import struct
import time
from profile import decode


class SwdBridge:
    def __init__(self, profiles):
        # Several firmware projects can share the same g_debug ABI. The board
        # decides which one is running: every candidate is compared against
        # Flash and only a fully matching AXF is used for addresses and fields.
        self.profiles = list(profiles) if isinstance(profiles, (list, tuple)) else [profiles]
        self.profile = self.profiles[0]
        self.session = None
        self.target = None

    @staticmethod
    def probes():
        from pyocd.core.helpers import ConnectHelper
        return [{'id': p.unique_id, 'name': p.description}
                for p in ConnectHelper.get_all_connected_probes(blocking=False)]

    def connect(self, probe_id):
        from pyocd.core.helpers import ConnectHelper
        try:
            self.session = ConnectHelper.session_with_chosen_probe(
                unique_id=probe_id, blocking=False, target_override='stm32h743xx',
                connect_mode='attach', frequency=1000000,
                options={'resume_on_disconnect': False, 'auto_unlock': False,
                         'allow_no_cores': False, 'cache.enable_memory': False,
                         'cache.enable_register': False,
                         'cmsis_dap.limit_packets': True,
                         'cmsis_dap.deferred_transfers': False})
            if not self.session:
                raise RuntimeError('未找到调试器，请先退出 Keil Debug，再刷新设备')
            self.session.open()
            self.target = self.session.target
            self.ensure_running()
            self.profile = self.identify()
            first = self.read()
            time.sleep(0.06)
            second = self.read()
            if first['system']['loop_heartbeat'] == second['system']['loop_heartbeat']:
                raise RuntimeError('H743 主循环没有递增，请先在 Keil 中确认 Run')
            if second['system']['fatal'] or second['system']['boot_stage'] != 9:
                raise RuntimeError('H743 尚未正常运行，请检查启动阶段和 fatal')
            return second
        except Exception:
            self.close()
            raise

    def ensure_running(self):
        from pyocd.core.target import Target
        if self.target.get_state() != Target.State.RUNNING:
            raise RuntimeError('CPU 未运行；请先在 Keil 中 Run 并退出调试。上位机不会自动复位/运行 CPU')

    def identify(self):
        """Choose the candidate AXF whose full Flash identity matches the board.

        A short prefix of the first load segment rejects the wrong project in a
        few hundred bytes; the chosen candidate is then verified completely, so
        a stale or partially written image can never be controlled."""
        if len(self.profiles) == 1:
            self.verify(self.profiles[0])
            return self.profiles[0]
        for profile in self.profiles:
            segment = profile['segments'][0]
            expected = bytes.fromhex(segment['data'])
            probe = min(len(expected), 512)
            actual = bytes(self.target.read_memory_block8(segment['address'], probe))
            if actual != expected[:probe]:
                continue
            self.verify(profile)
            return profile
        raise RuntimeError('板上固件与任何工程 AXF 都不一致，禁止控制；请重新编译并下载对应工程固件')

    def verify(self, profile):
        """Compare every Flash load segment, not just a vector or RAM address."""
        for segment in profile['segments']:
            expected = bytes.fromhex(segment['data'])
            actual = bytes(self.target.read_memory_block8(segment['address'], len(expected)))
            if actual != expected:
                raise RuntimeError('板上固件与 AXF 不一致，禁止控制；请使用对应工程生成的固件')

    def read(self):
        self.ensure_running()
        p = self.profile
        addr = p['address']
        result = {'diagnostics_coherent': False}
        fields=p['schema']['fields']
        for slot,name in enumerate(('motor','motor2')):
            if name not in fields:
                continue
            motor=fields[name]
            core_size=motor['fields']['can']['offset']
            coherent=False
            for _ in range(16):
                raw=bytes(self.target.read_memory_block8(addr+motor['offset'],core_size))
                seq=self.target.read32(addr+motor['offset'])
                if not int.from_bytes(raw[:4],'little') & 1 and seq==int.from_bytes(raw[:4],'little'):
                    coherent=True
                    break
            tail=bytes(self.target.read_memory_block8(addr+motor['offset']+core_size,motor['size']-core_size))
            result[name]=decode(motor,raw+tail)
            result['coherent' if slot==0 else 'coherent2']=coherent
        for name in ('cmd','cmd2','oled','system','setup','drive'):
            if name not in fields:
                continue
            field=fields[name]
            raw=bytes(self.target.read_memory_block8(addr+field['offset'],field['size']))
            result[name]=decode(field,raw)
        for slot,name in enumerate(('motor','motor2')):
            if name not in result:
                continue
            age=(result['system']['tick_ms']-result[name]['published_tick_ms']) & 0xFFFFFFFF
            result['publication_age_ms' if slot==0 else 'publication_age2_ms']=age
            for field in ('feedback_age_ms','current_age_ms','voltage_age_ms'):
                result[name][field]=min(0xFFFFFFFF,result[name][field]+age)
        if 'chassis' in fields:
            field=fields['chassis'];base=addr+field['offset']
            coherent=False
            for _ in range(8):
                raw=bytes(self.target.read_memory_block8(base,field['size']))
                seq=int.from_bytes(raw[:4],'little')
                if not seq&1 and seq==self.target.read32(base):
                    coherent=True;break
            result['chassis']=decode(field,raw)
            result['chassis_coherent']=coherent
        if 'led' in p:
            led=p['led']
            result['led']=decode(led['schema'],bytes(self.target.read_memory_block8(
                led['address'],led['schema']['size'])))
        return result

    def submit(self, command, params, motor=0):
        self.ensure_running()
        name='cmd2' if motor==1 else 'cmd'
        if type(motor) is not int or motor not in (0,1) or name not in self.profile['schema']['fields']:
            raise ValueError('固件不支持此电机通道')
        addr = self.profile['address']+self.profile['schema']['fields'][name]['offset']
        current = self.target.read32(addr)
        if current & 1 and command not in (5, 6):
            raise RuntimeError('命令区正在编辑；请关闭其他 Watch 写入或先失能')
        seq = ((current + (1 if current & 1 else 2)) & 0xFFFFFFFF) or 2
        # Commit ordering: odd -> complete payload -> flush -> fresh even LAST.
        self.target.write32(addr, seq - 1)
        self.target.flush()
        values = struct.pack('<IIffff', command, 0x52533035, params['target_rad_s'],
                             params['speed_limit_rad_s'], params['current_limit_a'], params['accel_rad_s2'])
        self.target.write_memory_block8(addr + 4, list(values))
        self.target.flush()
        if bytes(self.target.read_memory_block8(addr + 4, 24)) != values:
            raise RuntimeError('命令写入校验失败，未提交偶数序号')
        self.target.write32(addr, seq)
        self.target.flush()
        return seq

    def drive(self, command, targets=(0.0,0.0)):
        self.ensure_running()
        fields=self.profile['schema']['fields']
        if 'drive_cmd' not in fields or 'drive' not in fields:
            raise ValueError('请先下载支持 WASD 的 H7 固件')
        addr=self.profile['address']+fields['drive_cmd']['offset']
        current=self.target.read32(addr)
        if current&1 and command!=3:
            raise RuntimeError('小车命令区正在编辑，请退出后重新进入模式')
        seq=((current+(1 if current&1 else 2)) & 0xFFFFFFFF) or 2
        values=struct.pack('<IIff',0x52533035,command,*targets)
        self.target.write32(addr,seq-1);self.target.flush()
        self.target.write_memory_block8(addr+4,list(values));self.target.flush()
        if bytes(self.target.read_memory_block8(addr+4,len(values)))!=values:
            raise RuntimeError('小车命令写入校验失败，未提交')
        self.target.write32(addr,seq);self.target.flush()
        status=self.profile['address']+fields['drive']['offset']
        deadline=time.monotonic()+0.3
        while time.monotonic()<deadline:
            # executed_seq is committed after result/active by the motor task.
            if self.target.read32(status)==seq:
                code=self.target.read32(status+4)
                if code:raise ValueError(f'H7 拒绝小车命令，结果码 {code}')
                return {'seq':seq}
            time.sleep(0.002)
        raise RuntimeError('小车命令未获 H7 确认，模式已停止；不会自动恢复')

    def chassis(self, command, config):
        self.ensure_running()
        fields=self.profile['schema']['fields']
        field=fields.get('chassis_cmd')
        if field is None:raise ValueError('请下载 JY901S 航向控制新固件')
        addr=self.profile['address']+field['offset']
        seq=((self.target.read32(addr)+2)&0xFFFFFFFE) or 2
        raw=bytearray(field['size']-4)
        values={**config,'key':0x52533035,'command':command}
        for name,schema in field['fields'].items():
            if name=='seq':continue
            struct.pack_into('<f' if schema['kind']=='float' else '<I',raw,schema['offset']-4,values[name])
        self.target.write32(addr,seq-1);self.target.flush()
        self.target.write_memory_block8(addr+4,list(raw));self.target.flush()
        if bytes(self.target.read_memory_block8(addr+4,len(raw)))!=raw:
            raise RuntimeError('PID 参数写入未通过校验')
        self.target.write32(addr,seq);self.target.flush()
        status=fields['chassis'];base=self.profile['address']+status['offset']
        receipt=base+status['fields']['executed_seq']['offset']
        deadline=time.monotonic()+0.4
        while time.monotonic()<deadline:
            before=self.target.read32(base)
            if not before&1 and self.target.read32(receipt)==seq:
                result=self.target.read32(base+status['fields']['result']['offset'])
                if self.target.read32(base)!=before:continue
                if result:raise ValueError(f'H7 拒绝 PID 命令，结果码 {result}')
                return {'seq':seq}
            time.sleep(0.002)
        raise RuntimeError('PID 命令未获固件回执')

    def setup(self, command, isolated=False):
        self.ensure_running()
        field=self.profile['schema']['fields'].get('setup')
        if field is None:
            raise ValueError('请下载 RTOS 双电机固件')
        addr=self.profile['address']+field['offset']
        current=self.target.read32(addr)
        seq=((current+(1 if current&1 else 2)) & 0xFFFFFFFF) or 2
        values=struct.pack('<IIIII',0x52533035,command,127,126,0x49534F4C if isolated else 0)
        self.target.write32(addr,seq-1)
        self.target.flush()
        self.target.write_memory_block8(addr+4,list(values))
        self.target.flush()
        if bytes(self.target.read_memory_block8(addr+4,20))!=values:
            raise RuntimeError('配置写入校验失败，未提交')
        self.target.write32(addr,seq)
        self.target.flush()
        return seq

    def stop_all(self):
        self.ensure_running()
        field=self.profile['schema']['fields'].get('stop_all_seq')
        if field is None:
            return self.submit(5,dict(target_rad_s=0,speed_limit_rad_s=33,current_limit_a=2.4,accel_rad_s2=20))
        addr=self.profile['address']+field['offset']
        seq=((self.target.read32(addr)+2) & 0xFFFFFFFE) or 2
        self.target.write32(addr,seq)
        self.target.flush()
        return seq

    def set_led(self, config):
        self.ensure_running()
        command=self.profile.get('led_command')
        if command is None:raise ValueError('当前固件不支持灯效控制，请下载新版 RTOS 固件')
        addr=command['address']
        current=self.target.read32(addr)
        if current&1:raise RuntimeError('灯效命令区正在编辑，请关闭其它 Watch 写入后重试')
        seq=((current+2)&0xFFFFFFFF) or 2
        modes=config['cycle_modes']
        values=struct.pack('<18I',config['enabled'],config['mode'],config['brightness'],
            config['speed_percent'],config['color_order'],config['red'],config['green'],config['blue'],
            len(modes),config['cycle_interval_ms'],*(modes+[0]*(8-len(modes))))
        self.target.write32(addr,seq-1);self.target.flush()
        self.target.write_memory_block8(addr+4,list(values));self.target.flush()
        if bytes(self.target.read_memory_block8(addr+4,len(values)))!=values:
            raise RuntimeError('灯效写入校验失败，未提交')
        self.target.write32(addr,seq);self.target.flush()
        led=self.profile['led']; fields=led['schema']['fields']
        deadline=time.monotonic()+0.7
        while time.monotonic()<deadline:
            if self.target.read32(led['address']+fields['executed_seq']['offset'])==seq:
                code=self.target.read32(led['address']+fields['result']['offset'])
                if code:raise ValueError('固件拒绝灯效参数')
                return {'seq':seq,'accepted':True}
            time.sleep(0.005)
        raise RuntimeError('灯效未获固件确认，请检查连接后读取当前设置；不会自动重试')

    def close(self):
        if self.session:
            session = self.session
            try:
                session.close()
            finally:
                try:
                    # pyOCD 0.45 can exit disconnect/flush on a native USB
                    # OSError before closing its CMSIS-DAP transport. Release
                    # this session's handle even then; otherwise reconnect sees
                    # "already open" until the entire application exits.
                    probe = session.probe
                    interface = getattr(getattr(probe, '_link', None), '_interface', None)
                    if probe.is_open and interface is not None:
                        interface.close()
                finally:
                    self.session = self.target = None
