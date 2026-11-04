"""LED configuration bounds and the real SWD mailbox commit contract."""
import copy
import json
from pathlib import Path
import unittest
from bridge import SwdBridge
from profile import decode
from server import validate_led

CONFIG=dict(enabled=1,mode=0,brightness=64,speed_percent=100,color_order=0,
            red=32,green=180,blue=255,cycle_interval_ms=1000,cycle_modes=[7,1,7])
PROFILE=json.loads((Path(__file__).resolve().parents[1]/'firmware_profile.json').read_text('utf-8'))

class Memory:
    def __init__(self,corrupt=False):self.words={};self.ops=[];self.corrupt=corrupt
    def read32(self,a):return int.from_bytes(bytes(self.words.get(a+i,0) for i in range(4)),'little')
    def write32(self,a,v):
        self.ops.append((a,v))
        self.write_memory_block8(a,v.to_bytes(4,'little'))
        if a==PROFILE['led_command']['address'] and not v&1:
            addr=PROFILE['led']['address']+PROFILE['led']['schema']['fields']['executed_seq']['offset']
            self.write_memory_block8(addr,v.to_bytes(4,'little'))
    def write_memory_block8(self,a,b):self.words.update((a+i,v) for i,v in enumerate(b))
    def read_memory_block8(self,a,n):
        raw=[self.words.get(a+i,0) for i in range(n)]
        if self.corrupt and n==72:raw[0]^=1
        return raw
    def flush(self):pass

class LedTests(unittest.TestCase):
    def test_bounds_and_repeated_playlist(self):
        self.assertEqual(validate_led(CONFIG),CONFIG)
        for key,value in [('enabled',True),('mode',8),('brightness',256),('speed_percent',0),
                          ('cycle_interval_ms',999),('cycle_modes',[]),('cycle_modes',[0]),
                          ('cycle_modes',[1]*9),('red',1.0)]:
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                validate_led({**CONFIG,key:value})
        with self.assertRaises(ValueError):validate_led({})
    def test_commit_does_not_touch_motor_memory_and_waits_for_receipt(self):
        b=SwdBridge(PROFILE);b.target=Memory();b.ensure_running=lambda:None
        result=b.set_led(CONFIG);self.assertTrue(result['accepted'])
        a=PROFILE['led_command']['address']
        self.assertEqual(b.target.ops,[(a,1),(a,2)])
        command=decode(PROFILE['led_command']['schema'],bytes(b.target.read_memory_block8(a,76)))
        self.assertEqual(command['cycle_modes'][:3],[7,1,7])
        self.assertEqual(command['cycle_count'],3)
        lo=PROFILE['address'];hi=lo+PROFILE['schema']['size']
        self.assertFalse(any(lo<=address<hi for address in b.target.words))
    def test_failed_readback_never_commits_even_sequence(self):
        b=SwdBridge(PROFILE);b.target=Memory(corrupt=True);b.ensure_running=lambda:None
        with self.assertRaises(RuntimeError):b.set_led(CONFIG)
        self.assertEqual([v for a,v in b.target.ops],[1])
    def test_old_firmware_led_control_explicitly_unavailable(self):
        p=copy.deepcopy(PROFILE);p.pop('led_command')
        b=SwdBridge(p);b.ensure_running=lambda:None
        with self.assertRaises(ValueError):b.set_led(CONFIG)

if __name__=='__main__':unittest.main()
