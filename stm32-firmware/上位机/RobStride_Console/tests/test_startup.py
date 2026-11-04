"""Connection prepares both real mailbox channels without issuing movement."""
import copy
import time
import unittest
from unittest.mock import patch

from test_dual import FakeBridge, PROFILE
from server import Controller, DEFAULTS, validate


class StartupTests(unittest.TestCase):
    def controller(self):
        c=Controller(PROFILE);c.running=False;c.worker.join(1)
        c.bridge=FakeBridge();c.connected=True
        for name in ('motor','motor2'):
            c.bridge.snap[name].update(state=0,identity_valid=1)
        return c

    def begin(self,c):
        c.start_motors([validate(1,{}),validate(1,{'current_limit_a':3})],c.generation)
        c.service_startup(c.bridge.read())
        self.assertEqual(c.startup['phase'],'identity')
        c.bridge.snap['setup'].update(executed_seq=2,result=0,dual_ready=1)
        c.service_startup(c.bridge.read())
        self.assertEqual(c.startup['phase'],'prepare')

    def receipt(self,c,slot,state):
        c.bridge.snap[('motor','motor2')[slot]].update(executed_seq=2,result=0,
            state=state,mode=2,enabled=int(state==2),enable_requested=int(state==2))

    def test_connection_uses_both_drafts_and_forces_zero(self):
        c=self.controller();bridge=c.bridge
        bridge.profile=PROFILE;bridge.connect=lambda probe:bridge.read()
        with patch('server.SwdBridge') as cls:
            cls.probes.return_value=[{'id':'fixture'}];cls.return_value=bridge
            c.connected=False
            result=c.perform('connect',{'probe_id':'fixture','motor_params':[
                {'target_rad_s':5,'speed_limit_rad_s':12}, {'current_limit_a':3}]})
        self.assertTrue(result['initializing'])
        self.assertEqual(c.startup['params'][0]['speed_limit_rad_s'],12)
        self.assertEqual(c.startup['params'][1]['current_limit_a'],3)
        self.assertEqual([p['target_rad_s'] for p in c.startup['params']],[0,0])
        self.assertEqual(bridge.sent,[('all',5)])

    def test_order_independent_receipts_and_zero_speed_ready(self):
        c=self.controller();self.begin(c)
        self.assertEqual([s[:2] for s in c.bridge.sent],[('all',5),('setup',1),(0,1),(1,1)])
        self.receipt(c,0,1);c.service_startup(c.bridge.read())
        self.assertEqual(c.startup['phase'],'prepare')
        self.receipt(c,1,1);c.service_startup(c.bridge.read())
        self.assertEqual(c.startup['phase'],'enable')
        self.assertEqual([s[:2] for s in c.bridge.sent][-2:],[(0,2),(1,2)])
        # Equal receipt numbers belong to independent channels.
        c.bridge.snap['motor2']['executed_seq']=0
        self.receipt(c,0,2);c.service_startup(c.bridge.read())
        self.assertIsNotNone(c.startup)
        self.receipt(c,1,2);c.service_startup(c.bridge.read())
        self.assertIsNone(c.startup)
        c.snapshot=c.bridge.read();self.assertTrue(c.status()['motors_ready'])
        params=[s[2] for s in c.bridge.sent if isinstance(s[0],int)]
        self.assertTrue(all(p['target_rad_s']==0 for p in params))
        self.assertEqual([p['current_limit_a'] for p in params],[validate(1,{})['current_limit_a'],3]*2)

    def test_prepare_failure_stops_pair_and_never_enables(self):
        c=self.controller();self.begin(c)
        for slot in (0,1):self.receipt(c,slot,1)
        c.bridge.snap['motor2']['result']=7
        c.service_startup(c.bridge.read())
        self.assertIsNone(c.startup);self.assertEqual(c.bridge.sent[-1],('all',5))
        self.assertFalse(any(s[:2] in ((0,2),(1,2)) for s in c.bridge.sent))

    def test_offline_timeout_does_not_enable_or_repeat(self):
        c=self.controller();c.bridge.snap['motor2']['online']=0
        c.start_motors([dict(DEFAULTS),dict(DEFAULTS)],c.generation)
        c.service_startup(c.bridge.read());self.assertEqual(c.startup['phase'],'stopping')
        c.startup['at']=time.monotonic()-6;c.service_startup(c.bridge.read())
        c.bridge.snap['motor2']['online']=1
        for _ in range(4):c.service_startup(c.bridge.read())
        self.assertEqual(c.bridge.sent,[('all',5),('all',5)])
        self.assertIn('失败',c.startup_message)

    def test_stop_at_any_phase_cancels_without_rearming(self):
        for action,payload in [('stop_all',{}),('command',{'command':5}),
                ('command',{'command':4}),('chassis',{'command':4})]:
            with self.subTest(action=action,payload=payload):
                c=self.controller();self.begin(c)
                c.perform(action,payload)
                for slot in (0,1):self.receipt(c,slot,1)
                for _ in range(3):c.service_startup(c.bridge.read())
                self.assertIsNone(c.startup);self.assertEqual(c.bridge.sent[-1],('all',5))
                self.assertFalse(any(s[:2] in ((0,2),(1,2)) for s in c.bridge.sent))

    def test_queued_stop_prevents_next_startup_phase(self):
        c=self.controller();self.begin(c)
        c.enqueue('stop_all',{})
        for slot in (0,1):self.receipt(c,slot,1)
        c.service_startup(c.bridge.read())
        self.assertIsNone(c.startup)
        self.assertFalse(any(s[:2] in ((0,2),(1,2)) for s in c.bridge.sent))

    def test_manual_motion_waits_for_startup(self):
        c=self.controller();self.begin(c)
        with self.assertRaisesRegex(ValueError,'自动配置'):
            c.perform('command',{'command':3,'target_rad_s':1})


if __name__=='__main__':unittest.main()
