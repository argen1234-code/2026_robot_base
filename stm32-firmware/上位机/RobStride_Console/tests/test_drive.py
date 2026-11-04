"""WASD contracts using production controller and SWD writer, without hardware."""
import time
import unittest
from bridge import SwdBridge
from profile import decode
from server import LIMITS
import test_dual as dual

PROFILE=dual.PROFILE


class DriveTests(unittest.TestCase):
    def controller(self):
        c=dual.DualTests().controller()
        for name in ('motor','motor2'):
            c.bridge.snap[name].update(state=2,enabled=1,enable_requested=1)
        c.bridge.snap['drive'].update(active=0)
        def drive(command,targets=(0,0)):
            c.bridge.sent.append(('drive',command,targets))
            c.bridge.snap['drive']['active']=int(command!=3)
            return {'seq':2}
        c.bridge.drive=drive
        return c

    def enter(self,c):
        return c.perform('drive_enter',{})['owner']

    def test_acquire_and_full_existing_speed_range_without_extra_cap(self):
        c=self.controller();owner=self.enter(c)
        c.perform('drive_update',{'owner':owner,'targets':[33,-33]})
        self.assertEqual(c.drive_targets,(33,-33))
        self.assertEqual(LIMITS['speed_limit_rad_s'],33)
        self.assertEqual(c.bridge.sent[-1],('drive',2,(33,-33)))
        with self.assertRaises(ValueError):c.perform('command',{'command':3,'confirmed':True})
        c.perform('drive_exit',{'owner':owner})
        self.assertIsNone(c.drive_owner)
        self.assertEqual(c.bridge.sent[-1][1],3)

    def test_invalid_second_wheel_zeros_both_and_never_clamps(self):
        for bad in (33.01,True,float('nan'),float('inf'),'1'):
            c=self.controller();owner=self.enter(c)
            with self.assertRaises(ValueError):c.perform('drive_update',{'owner':owner,'targets':[1,bad]})
            self.assertIsNone(c.drive_owner)
            self.assertEqual(c.bridge.sent[-1],('drive',3,(0,0)))
        c=self.controller();owner=self.enter(c)
        c.bridge.snap['motor2']['speed_limit_rad_s']=2
        with self.assertRaises(ValueError):c.perform('drive_update',{'owner':owner,'targets':[3,3]})
        self.assertFalse(any(s[1]==2 for s in c.bridge.sent))

    def test_client_lease_stops_and_late_or_other_tab_cannot_resume(self):
        c=self.controller();owner=self.enter(c)
        c.drive_client_at=time.monotonic()-1
        c.service_drive()
        self.assertIsNone(c.drive_owner)
        with self.assertRaises(ValueError):c.perform('drive_update',{'owner':owner,'targets':[1,1]})
        new=self.enter(c)
        for action in ('drive_update','drive_exit'):
            with self.assertRaises(ValueError):c.perform(action,{'owner':owner,'targets':[1,1]})
        self.assertEqual(c.drive_owner,new)

    def test_heartbeat_renews_pair_and_escape_preempts_queued_vectors(self):
        c=self.controller();owner=self.enter(c)
        c.drive_sent_at=time.monotonic()-0.2
        c.service_drive()
        self.assertEqual(c.bridge.sent[-1],('drive',2,(0,0)))
        update=c.enqueue('drive_update',{'owner':owner,'targets':[1,1]})
        exit_ticket=c.enqueue('drive_exit',{'owner':owner})
        self.assertIs(c.jobs.get_nowait()[2],exit_ticket)
        self.assertNotEqual(update['generation'],c.generation)
        c.perform('stop_all',{})
        self.assertIsNone(c.drive_owner)

    def test_prepared_enabled_and_pending_requirements(self):
        for name,change in (('motor',{'state':1}),('motor2',{'enabled':0}),
                            ('motor2',{'state':5}),('motor',{'job_command':2})):
            c=self.controller();c.bridge.snap[name].update(change)
            with self.assertRaises(ValueError):self.enter(c)
            self.assertFalse(c.bridge.sent)
        c=self.controller();c.pending={'seq':2}
        with self.assertRaises(ValueError):self.enter(c)

    def test_acquire_accepts_feedback_noise_running_and_stopping_without_duplicate_confirmations(self):
        for state in (2,3,4):
            c=self.controller()
            c.bridge.snap['motor'].update(state=state,target_rad_s=1,applied_rad_s=0.7,
                velocity_rad_s=0.065,timeout_motor_supported=0)
            c.bridge.snap['motor2'].update(state=state,target_rad_s=-1,applied_rad_s=-0.7,
                velocity_rad_s=-0.08,timeout_motor_supported=0)
            self.enter(c)
            self.assertEqual(c.bridge.sent,[('drive',1,(0,0))])
            self.assertIsNotNone(c.drive_owner)

    def test_swd_pair_commits_once_and_torn_payload_never_commits(self):
        b=SwdBridge(PROFILE);b.target=dual.Target();b.ensure_running=lambda:None
        target=b.target
        status=PROFILE['schema']['fields']['drive']['offset']
        original=target.write32
        def commit(address,value):
            original(address,value)
            at=PROFILE['schema']['fields']['drive_cmd']['offset']
            if address==PROFILE['address']+at and value and not value&1:
                target.memory[status:status+4]=value.to_bytes(4,'little')
        target.write32=commit
        b.drive(2,(33,-33))
        snap=decode(PROFILE['schema'],target.memory)
        self.assertEqual(snap['drive_cmd']['motor1_rad_s'],33)
        self.assertEqual(snap['drive_cmd']['motor2_rad_s'],-33)
        self.assertEqual(snap['cmd']['seq'],0)
        self.assertEqual(snap['cmd2']['seq'],0)
        target.read_memory_block8=lambda *_:bytes(16)
        with self.assertRaises(RuntimeError):b.drive(2,(1,1))
        self.assertTrue(target.read32(PROFILE['address']+PROFILE['schema']['fields']['drive_cmd']['offset'])&1)


if __name__=='__main__':unittest.main()
