"""Chassis control contracts against the production controller and DWARF ABI."""
import struct
import copy
import unittest
from bridge import SwdBridge
from profile import decode
from server import DEFAULTS
import test_dual as dual

PROFILE=dual.PROFILE
CONFIG=dict(left_slot=1,left_sign=-1.,right_sign=1.,yaw_sign=1.,target_deg=90.,
    left_rad_s=1.,right_rad_s=1.,turn_limit_rad_s=2.,angle_kp=.04,angle_ki=0.,angle_kd=.005)

class ChassisTests(unittest.TestCase):
    def controller(self):
        c=dual.DualTests().controller()
        c.bridge.snap['chassis']['config'].update(CONFIG)
        c.bridge.chassis=lambda command,config:c.bridge.sent.append(('chassis',command,config)) or {'seq':2}
        return c

    def test_prepare_and_enable_both_with_same_speed_mode_and_existing_limits(self):
        c=self.controller()
        for name in ('motor','motor2'):c.bridge.snap[name].update(state=0,enable_requested=0)
        c.perform('chassis_prepare',{'current_limit_a':1.5,'speed_limit_rad_s':8,'accel_rad_s2':20})
        self.assertEqual([v[:2] for v in c.bridge.sent],[(0,1),(1,1)])
        self.assertTrue(all(v[2]['current_limit_a']==1.5 for v in c.bridge.sent))
        c.pending=c.pending2=None
        for name in ('motor','motor2'):c.bridge.snap[name].update(state=1,mode=2)
        c.perform('chassis_enable',{})
        self.assertEqual([v[:2] for v in c.bridge.sent[-2:]],[(0,2),(1,2)])

    def test_pair_prepare_prechecks_both_before_either_write(self):
        c=self.controller();c.bridge.snap['motor2'].update(state=2,enable_requested=1)
        with self.assertRaises(ValueError):c.perform('chassis_prepare',{})
        self.assertEqual(c.bridge.sent,[])

    def test_partial_tuning_preserves_other_gains_and_mounting(self):
        c=self.controller();c.perform('chassis',{'command':1,'config':{'angle_kp':.08}})
        sent=c.bridge.sent[-1]
        self.assertEqual(sent[2]['angle_kp'],.08)
        self.assertEqual(sent[2]['angle_kd'],.005)
        self.assertEqual(sent[2]['left_slot'],1)
        self.assertEqual(sent[2]['left_sign'],-1)
        with self.assertRaises(ValueError):c.perform('chassis',{'command':1,'config':{'angle_kp':float('nan')}})

    def test_stop_preempts_queued_heading(self):
        c=self.controller();move=c.enqueue('chassis',{'command':3});stop=c.enqueue('chassis',{'command':4})
        self.assertIs(c.jobs.get_nowait()[2],stop)
        self.assertNotEqual(move['generation'],c.generation)

    def test_worker_records_observation_and_gains_for_csv(self):
        c=self.controller()
        c.bridge.snap['chassis_coherent']=True
        c.bridge.snap['system'].update(tick_ms=20,boot_stage=9)
        c.bridge.snap['chassis'].update(published_tick_ms=20,yaw_deg=25)
        def read():
            c.running=False
            return copy.deepcopy(c.bridge.snap)
        c.bridge.read=read;c.running=True;c.loop()
        row=c.chassis_history[-1]
        self.assertEqual(row['yaw_deg'],25)
        self.assertEqual(row['config_left_sign'],-1)
        self.assertIn('imu_angle_frames',row)

    def test_swd_only_commits_chassis_mailbox_and_waits_for_stable_receipt(self):
        b=SwdBridge(PROFILE);b.target=dual.Target();b.ensure_running=lambda:None
        fields=PROFILE['schema']['fields'];offset=fields['chassis_cmd']['offset'];status=fields['chassis']['offset']
        original=b.target.write32
        def write(address,value):
            original(address,value)
            if address==PROFILE['address']+offset and value and not value&1:
                struct.pack_into('<III',b.target.memory,status,4,value,0)
        b.target.write32=write
        result=b.chassis(3,CONFIG)
        memory=decode(PROFILE['schema'],b.target.memory)
        self.assertEqual(memory['chassis_cmd']['seq'],result['seq'])
        self.assertEqual(memory['chassis_cmd']['target_deg'],90)
        self.assertEqual(memory['chassis_cmd']['left_sign'],-1)
        self.assertEqual(bytes(b.target.memory[:offset]),bytes(offset))
        self.assertEqual(memory['cmd']['seq'],0)
        self.assertEqual(memory['cmd2']['seq'],0)

if __name__=='__main__':unittest.main()
