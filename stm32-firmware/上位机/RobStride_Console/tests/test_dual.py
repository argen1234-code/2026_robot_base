"""Dual channel contracts against the real AXF schema; no target access."""
import copy
import json
from pathlib import Path
import threading
import unittest

from bridge import SwdBridge
from profile import decode
from server import Controller,DEFAULTS,validate

PROFILE=json.loads((Path(__file__).resolve().parents[1]/'firmware_profile.json').read_text('utf-8'))

class FakeBridge:
    def __init__(self):
        self.sent=[]
        self.snap=decode(PROFILE['schema'],bytes(PROFILE['schema']['size']))
        self.snap.update(coherent=True,coherent2=True)
        self.snap['setup']['dual_ready']=1
        for name in ('motor','motor2'):
            self.snap[name].update(state=1,online=1,feedback_age_ms=1,timeout_motor_supported=1,**validate(1,{}))
    def read(self):return copy.deepcopy(self.snap)
    def submit(self,command,params,motor=0):
        self.sent.append((motor,command,dict(params)))
        return 2
    def stop_all(self):self.sent.append(('all',5));return 2
    def close(self):pass
    def setup(self,command,isolated):self.sent.append(('setup',command,isolated));return 2

class Target:
    def __init__(self):self.memory=bytearray(PROFILE['schema']['size']);self.ops=[]
    def read32(self,address):
        at=address-PROFILE['address'];return int.from_bytes(self.memory[at:at+4],'little')
    def write32(self,address,value):
        at=address-PROFILE['address'];self.memory[at:at+4]=value.to_bytes(4,'little');self.ops.append((at,value))
    def flush(self):pass
    def read_memory_block8(self,address,size):
        at=address-PROFILE['address'];return self.memory[at:at+size]
    def write_memory_block8(self,address,values):
        at=address-PROFILE['address'];self.memory[at:at+len(values)]=bytes(values)

class DualTests(unittest.TestCase):
    def controller(self):
        c=Controller(PROFILE);c.running=False;c.worker.join(1)
        c.bridge=FakeBridge();c.connected=True;return c
    def test_second_command_writes_only_secondary_mailbox(self):
        b=SwdBridge(PROFILE);b.target=Target();b.ensure_running=lambda:None
        b.submit(3,{**DEFAULTS,'target_rad_s':-1},motor=1)
        s=decode(PROFILE['schema'],b.target.memory)
        self.assertEqual(s['cmd']['seq'],0)
        self.assertEqual(s['cmd2']['seq'],2)
        self.assertEqual(s['cmd2']['target_rad_s'],-1)
        self.assertEqual(b.target.ops[0][0],PROFILE['schema']['fields']['cmd2']['offset'])
    def test_setup_commits_only_host_fields_and_stop_all_only_one_word(self):
        b=SwdBridge(PROFILE);b.target=Target();b.ensure_running=lambda:None
        b.setup(2,isolated=True)
        s=decode(PROFILE['schema'],b.target.memory)
        self.assertEqual(s['setup']['isolated_key'],0x49534F4C)
        self.assertEqual(s['setup']['old_id'],127);self.assertEqual(s['setup']['new_id'],126)
        self.assertFalse(s['setup']['dual_ready']);self.assertFalse(s['cmd']['seq']);self.assertFalse(s['cmd2']['seq'])
        before=bytes(b.target.memory);b.stop_all();after=bytes(b.target.memory)
        at=PROFILE['schema']['fields']['stop_all_seq']['offset']
        self.assertEqual(before[:at],after[:at]);self.assertEqual(before[at+4:],after[at+4:])
    def test_simultaneous_receipts_with_identical_seq_are_independent(self):
        c=self.controller()
        c.perform('command',{'command':2,'motor':0,'confirmed':True})
        c.perform('command',{'command':2,'motor':1,'confirmed':True})
        self.assertEqual([s[:2] for s in c.bridge.sent],[(0,2),(1,2)])
        c.bridge.snap['system'].update(boot_stage=9,tick_ms=2)
        c.bridge.snap['motor'].update(executed_seq=2,result=0)
        def read():c.running=False;return copy.deepcopy(c.bridge.snap)
        c.bridge.read=read;c.running=True;c.loop()
        self.assertIsNone(c.pending);self.assertEqual(c.pending2['seq'],2)
        self.assertEqual({r['motor'] for r in c.history},{1,2})
    def test_inconsistent_motor2_does_not_block_motor1(self):
        c=self.controller();c.bridge.snap['coherent2']=False
        c.perform('command',{'command':2,'motor':0,'confirmed':True})
        with self.assertRaises(ValueError):c.perform('command',{'command':2,'motor':1,'confirmed':True})
        self.assertEqual(len(c.bridge.sent),1)
    def test_unverified_pair_blocks_motion_but_all_stop_is_available(self):
        c=self.controller();c.bridge.snap['setup']['dual_ready']=0
        for slot in (0,1):
            with self.assertRaises(ValueError):c.perform('command',{'command':2,'motor':slot,'confirmed':True})
        c.perform('stop_all',{});self.assertEqual(c.bridge.sent,[('all',5)])
    def test_id_change_requires_isolation_and_both_disabled(self):
        c=self.controller()
        for name in ('motor','motor2'):c.bridge.snap[name]['state']=0
        with self.assertRaises(ValueError):c.perform('setup',{'command':2})
        c.bridge.snap['motor2']['enable_requested']=1
        with self.assertRaises(ValueError):c.perform('setup',{'command':2,'isolated':True})
        self.assertFalse(c.bridge.sent)
        c.bridge.snap['motor2']['enable_requested']=0
        c.perform('setup',{'command':2,'isolated':True})
        self.assertEqual(c.bridge.sent,[('setup',2,True)])
    def test_all_stop_preempts_both_queued_movement_channels(self):
        c=self.controller()
        first=c.enqueue('command',{'command':3,'motor':0})
        second=c.enqueue('command',{'command':3,'motor':1})
        stop=c.enqueue('stop_all',{})
        self.assertIs(c.jobs.get_nowait()[2],stop)
        for ticket in (first,second):self.assertNotEqual(ticket['generation'],c.generation)
    def test_bad_slot_never_writes(self):
        c=self.controller()
        for slot in (True,-1,2,'1',None):
            with self.assertRaises(ValueError):c.perform('command',{'command':5,'motor':slot})
        self.assertFalse(c.bridge.sent)
    def test_disconnect_stops_both(self):
        c=self.controller();c.perform('disconnect',{})
        self.assertEqual(c.bridge, None);self.assertFalse(c.connected)

if __name__=='__main__':unittest.main()
