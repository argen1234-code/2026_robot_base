"""Contract tests for the production SWD commit, ABI, bounds, and state gates.
No physical target is attached by these tests.
"""
import copy
import json
from pathlib import Path
import struct
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge import SwdBridge
from profile import decode
from server import Controller, DEFAULTS, validate

PROFILE = json.loads((Path(__file__).resolve().parents[1] / 'firmware_profile.json').read_text('utf-8'))


class MemoryTarget:
    def __init__(self):
        self.memory = bytearray(28)
        self.ops = []

    def read32(self, address):
        return int.from_bytes(self.memory[:4], 'little')

    def write32(self, address, value):
        self.ops.append(('seq', value))
        self.memory[:4] = value.to_bytes(4, 'little')

    def flush(self):
        self.ops.append(('flush',))

    def write_memory_block8(self, address, values):
        self.ops.append(('payload', len(values)))
        self.memory[4:] = bytes(values)

    def read_memory_block8(self, address, length):
        return self.memory[4:4 + length]


class FakeBridge:
    def __init__(self):
        self.sent = []
        raw = bytes(PROFILE['schema']['size'])
        self.snap = decode(PROFILE['schema'], raw)
        self.snap['coherent'] = True
        self.snap['coherent2'] = True
        if 'setup' in self.snap:self.snap['setup']['dual_ready']=1
        self.snap['motor'].update(state=1, online=1, feedback_age_ms=1, timeout_readback=5000, timeout_motor_supported=1, **DEFAULTS)

    def read(self):
        return copy.deepcopy(self.snap)

    def submit(self, command, params):
        self.sent.append((command, params))
        return 2


class ControlTests(unittest.TestCase):
    def test_native_usb_error_still_releases_owned_probe_handle(self):
        from types import SimpleNamespace
        closed=[]
        interface=SimpleNamespace(close=lambda:closed.append(True))
        probe=SimpleNamespace(is_open=True,_link=SimpleNamespace(_interface=interface))
        class BrokenSession:
            def close(self):
                raise OSError('USB I/O')
        session=BrokenSession();session.probe=probe
        b=SwdBridge(PROFILE);b.session=session;b.target=object()
        with self.assertRaises(OSError):b.close()
        self.assertEqual(closed,[True])
        self.assertIsNone(b.session);self.assertIsNone(b.target)

    def test_commit_odd_payload_flush_even_last_and_never_zero(self):
        b = SwdBridge(PROFILE)
        b.target = MemoryTarget()
        b.ensure_running = lambda: None
        self.assertEqual(b.submit(3, DEFAULTS), 2)
        self.assertEqual(b.target.ops, [('seq',1),('flush',),('payload',24),('flush',),('seq',2),('flush',)])
        self.assertEqual(struct.unpack('<IIIffff', b.target.memory)[:3], (2,3,0x52533035))
        b.target.memory[:4] = (0xFFFFFFFE).to_bytes(4,'little')
        self.assertEqual(b.submit(5, DEFAULTS), 2)

    def test_torn_payload_is_never_committed(self):
        b = SwdBridge(PROFILE)
        b.target = MemoryTarget()
        b.ensure_running = lambda: None
        b.target.read_memory_block8 = lambda *_: bytes(24)
        with self.assertRaises(RuntimeError):
            b.submit(3, DEFAULTS)
        self.assertEqual(b.target.read32(0), 1)
        self.assertNotIn(('seq', 2), b.target.ops)

    def test_motion_snapshot_checks_its_sequence_independent_of_diagnostics(self):
        b = SwdBridge(PROFILE)
        b.ensure_running = lambda: None
        base = PROFILE['address']
        m = PROFILE['schema']['fields']['motor']
        reads=[]
        class SampleTarget:
            def read_memory_block8(self, address, length):
                reads.append((address,length))
                raw=bytearray(length)
                if address == base + m['offset']:
                    raw[:4]=(2).to_bytes(4,'little')
                    offset=m['fields']['published_tick_ms']['offset']
                    raw[offset:offset+4]=(0xFFFFFFF0).to_bytes(4,'little')
                elif address == base + PROFILE['schema']['fields']['system']['offset']:
                    offset=PROFILE['schema']['fields']['system']['fields']['tick_ms']['offset']
                    raw[offset:offset+4]=(7).to_bytes(4,'little')
                return raw
            def read32(self, address):
                return 2
        b.target=SampleTarget()
        snapshot=b.read()
        self.assertTrue(snapshot['coherent'])
        self.assertFalse(snapshot['diagnostics_coherent'])
        self.assertEqual(snapshot['publication_age_ms'],23)
        self.assertEqual(snapshot['motor']['feedback_age_ms'],23)
        self.assertEqual(reads[0],(base+m['offset'],m['fields']['can']['offset']))
        b.target.read32=lambda _: 4
        self.assertFalse(b.read()['coherent'])

    def test_bounds_nan_bool_huge_and_safe_disable(self):
        for bad in (float('nan'), float('inf'), 33.01, -1, True, '0.2', 1e300):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                validate(1, {'speed_limit_rad_s':bad})
        with self.assertRaises(ValueError):
            validate(3, {'target_rad_s':33.01})
        with self.assertRaises(ValueError):
            validate(2, {'target_rad_s':.1})
        self.assertEqual(validate(5, {'target_rad_s':float('nan')}), DEFAULTS)

    def test_model_speed_current_and_acceleration_ranges(self):
        for speed in (33, -33, 1, -1):
            params=validate(3,dict(target_rad_s=speed,speed_limit_rad_s=33,current_limit_a=11,accel_rad_s2=150))
            self.assertEqual(params['target_rad_s'],speed)
        for key,value in (('target_rad_s',33.00001),('target_rad_s',-33.00001),
                          ('speed_limit_rad_s',33.00001),('current_limit_a',11.00001),
                          ('accel_rad_s2',150.00001)):
            with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                validate(3,{key:value})
        with self.assertRaises(ValueError):
            validate(3,dict(target_rad_s=2,speed_limit_rad_s=1))

    def controller(self):
        c = Controller(PROFILE)
        c.running=False
        c.worker.join(1)
        c.bridge=FakeBridge()
        c.connected=True
        return c

    def test_no_command_without_online_state_or_consistency(self):
        for patch in ({'state':0}, {'online':0}, {'feedback_age_ms':150}, {'fault':1}, {'health_fault':1}):
            c=self.controller();c.bridge.snap['motor'].update(patch)
            with self.assertRaises(ValueError):
                c.perform('command', {'command':2, 'confirmed':True})
            self.assertEqual(c.bridge.sent, [])
        c=self.controller()
        c.bridge.snap['motor'].update(validate(1, {}))
        c.bridge.snap['coherent']=False
        with self.assertRaises(ValueError):
            c.perform('command', {'command':2,'confirmed':True})
        self.assertEqual(c.bridge.sent, [])

    def test_float32_limits_match_prepared_and_exactly_one_submit(self):
        c=self.controller()
        c.bridge.snap['motor'].update(validate(1, {}))
        c.perform('command', {'command':2,'confirmed':True})
        self.assertEqual(len(c.bridge.sent),1)
        self.assertIsNotNone(c.pending)
        with self.assertRaises(ValueError):
            c.perform('command', {'command':2,'confirmed':True})
        self.assertEqual(len(c.bridge.sent),1)

    def test_urgent_disable_overrides_pending_and_invalid_fields(self):
        c=self.controller();c.pending={'seq':88}
        c.bridge.snap['motor'].update(state=7,job_command=2,online=0)
        c.perform('command', {'command':5,'target_rad_s':'invalid','continuous':'invalid'})
        self.assertEqual(c.bridge.sent, [(5,DEFAULTS)])

    def test_disable_priority_cancels_queued_movement(self):
        c=self.controller()
        old=c.enqueue('command', {'command':3})
        urgent=c.enqueue('command', {'command':5})
        priority,_,ticket=c.jobs.get_nowait()
        self.assertEqual(priority,0)
        self.assertIs(ticket,urgent)
        self.assertNotEqual(old['generation'],c.generation)

    def start_continuous(self):
        c=self.controller()
        params=validate(3,{'target_rad_s':.05})
        c.bridge.snap['motor'].update(state=2,enabled=1,enable_requested=1,**validate(1,{}))
        c.perform('command',{'command':3,'confirmed':True,'target_rad_s':.05})
        c.bridge.snap['motor'].update(state=3,executed_seq=2,result=0,**params)
        c.pending=None
        c.snapshot=c.bridge.read()
        return c

    def test_run_is_submitted_once_without_client_heartbeat_or_refresh(self):
        from unittest.mock import patch
        c=self.start_continuous()
        c.bridge.snap['system'].update(boot_stage=9,fatal=0,tick_ms=0)
        c.running=True
        reads=[]
        def read():
            reads.append(True)
            c.bridge.snap['system']['tick_ms']+=6000
            if len(reads)==5:c.running=False
            return copy.deepcopy(c.bridge.snap)
        c.bridge.read=read
        with patch('server.time.monotonic',side_effect=[10,16,22,28,34]):
            c.loop()
        self.assertEqual(len(c.bridge.sent),1)
        self.assertEqual(c.bridge.sent[0][0],3)
        self.assertTrue(c.status()['continuous'])

    def test_stop_disable_disconnect_invalidate_queued_runs(self):
        for action,payload in [('command',{'command':4}),('command',{'command':5}),('disconnect',{})]:
            c=self.start_continuous();old=c.generation
            c.enqueue(action,payload)
            self.assertGreater(c.generation,old)
        c=self.start_continuous();c.pending={'seq':2,'command':3}
        c.perform('command',{'command':4})
        self.assertEqual(c.bridge.sent[-1][0],4)

    def test_run_display_follows_firmware_and_never_restores_faults(self):
        for state,enabled in ((5,0),(0,0),(2,1),(4,1)):
            c=self.start_continuous()
            c.snapshot['motor'].update(state=state,enabled=enabled)
            self.assertFalse(c.status()['continuous'])
            self.assertEqual(len(c.bridge.sent),1)
        c=self.start_continuous();c.connected=False
        self.assertFalse(c.status()['continuous'])

    def test_legacy_timeout_does_not_require_manual_acknowledgement(self):
        c=self.controller();c.bridge.snap['motor'].update(timeout_readback=0,timeout_motor_supported=0,**validate(1,{}))
        c.perform('command',{'command':2})
        self.assertEqual(c.bridge.sent[0][0],2)


if __name__=='__main__':
    unittest.main(verbosity=2)
