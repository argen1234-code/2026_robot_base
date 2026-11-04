"""Exercise production serial worker with a fragmented UART peer; no hardware."""
import threading
import time
import unittest
from bluetooth import BluetoothController


class SerialPeer:
    def __init__(self, **kwargs):
        self.settings=kwargs;self.buffer=bytearray();self.sent=[]
        self.armed=False;self.moving=False;self.speed=1;self.closed=False
        self.lock=threading.Lock();self.failed=False;self.reject=False

    @property
    def in_waiting(self):
        return min(len(self.buffer),3)  # Force receipts across many serial reads.

    def reset_input_buffer(self):self.buffer.clear()
    def close(self):self.closed=True

    def read(self,count):
        if self.failed:raise OSError('peer disconnected')
        with self.lock:
            value=bytes(self.buffer[:count]);del self.buffer[:count]
        if not value:time.sleep(.001)
        return value

    def write(self,raw):
        if self.failed:raise OSError('peer disconnected')
        assert raw.startswith(b'@') and raw.endswith(b'\r\n')
        command=raw[1:-2].decode();self.sent.append(command)
        if command=='PING':reply='PONG RS05-BT/1'
        elif command=='STATUS':
            reply=f'STATE armed={int(self.armed)} phase=0 moving={int(self.moving)} speed={self.speed} result=0 v1=0.125 v2=-0.125'
        elif command=='!':self.armed=self.moving=False;reply='OK DISARM'
        elif command=='S':self.moving=False;reply='OK S'
        elif command=='ARM':self.armed=True;reply='PENDING ARM\r\nOK ARM'
        elif command.startswith('SPEED='):self.speed=float(command[6:]);reply='OK SPEED'
        elif command.startswith('MAP='):reply='OK MAP'
        elif command.startswith('LIMIT='):reply='OK LIMIT'
        elif command in ('F','B','L','R','Q','E','FL','FR','BL','BR'):
            if self.reject:reply='ERR 2'
            elif not self.armed:reply='ERR 3'
            else:self.moving=True;reply='OK MOVE'
        else:reply='ERR 1'
        with self.lock:self.buffer.extend((reply+'\r\n').encode())
        return len(raw)


class BluetoothTests(unittest.TestCase):
    def setUp(self):
        self.peers=[]
        def factory(**kwargs):
            peer=SerialPeer(**kwargs);self.peers.append(peer);return peer
        self.controller=BluetoothController(serial_factory=factory)
        self.session=self.controller.call('connect',{'port':'COM42'})['session']
        self.peer=self.peers[-1]

    def tearDown(self):self.controller.close()

    def call(self,action,**payload):
        return self.controller.call(action,{'session':self.session,**payload})

    def arm(self,**values):
        self.call('arm',cap=values.get('cap',33),current=2.4,accel=20,
                  speed=values.get('speed',1),left_slot=1,left_sign=-1,right_sign=1)
        deadline=time.monotonic()+1
        while not self.controller.status()['armed'] and time.monotonic()<deadline:time.sleep(.005)
        self.assertTrue(self.controller.status()['armed'])

    def test_handshake_and_explicit_arm_zero_only(self):
        self.assertEqual(self.peer.settings['baudrate'],9600)
        self.assertEqual(self.controller.status()['identity'],'RS05-BT/1')
        self.assertNotIn('ARM',self.peer.sent)
        self.arm(speed=33)
        for item in ('LIMIT=33,2.4,20','MAP=1,-1,1','SPEED=33','ARM'):
            self.assertIn(item,self.peer.sent)
        self.assertFalse(self.peer.moving)

    def test_direction_renewal_and_browser_loss_stop(self):
        self.arm()
        self.call('motion',direction='F',seq=1)
        time.sleep(.24)
        self.call('motion',direction='F',seq=2)
        time.sleep(.24)
        self.assertGreaterEqual(self.peer.sent.count('F'),3)
        time.sleep(.7)
        self.assertEqual(self.controller.direction,'S')
        self.assertFalse(self.peer.moving)
        self.assertEqual([x for x in self.peer.sent if x in ('F','S')][-1],'S')

    def test_stale_sequence_and_reconnect_session(self):
        self.arm();self.call('motion',direction='L',seq=10)
        self.call('motion',direction='S',seq=12)
        self.assertTrue(self.call('motion',direction='F',seq=11)['stale'])
        self.assertFalse(self.peer.moving)
        self.controller.call('connect',{'port':'COM43'})
        with self.assertRaises(ValueError):self.call('motion',direction='F',seq=13)
        self.assertTrue(self.peer.closed)
        self.assertIn('!',self.peer.sent)

    def test_stop_disable_and_speed_are_not_motion(self):
        self.arm();self.call('motion',direction='B',seq=1)
        self.call('speed',speed=2)
        self.assertFalse(self.peer.moving);self.assertEqual(self.peer.speed,2)
        self.controller.call('stop',{'disable':True})
        self.assertFalse(self.controller.status()['armed'])
        with self.assertRaises(ValueError):self.call('motion',direction='R',seq=2)

    def test_finite_limits_and_direction_validation(self):
        for speed in (float('nan'),float('inf'),33.01,True,'1'):
            with self.assertRaises(ValueError):self.arm(speed=speed)
        with self.assertRaises(ValueError):self.arm(cap=2,speed=3)
        self.arm()
        with self.assertRaises(ValueError):self.call('motion',direction='F\r\n@!',seq=1)
        self.assertNotIn('F',self.peer.sent)

    def test_rejection_revokes_host_repeat(self):
        self.arm();self.peer.reject=True
        with self.assertRaises(ValueError):self.call('motion',direction='F',seq=1)
        self.assertEqual(self.controller.direction,'S')
        count=self.peer.sent.count('F');time.sleep(.24)
        self.assertEqual(self.peer.sent.count('F'),count)

    def test_serial_error_disconnects_and_requires_new_handshake(self):
        self.arm();self.call('motion',direction='F',seq=1)
        self.peer.failed=True
        deadline=time.monotonic()+1
        while self.controller.status()['connected'] and time.monotonic()<deadline:time.sleep(.01)
        self.assertFalse(self.controller.status()['connected'])
        self.assertFalse(self.controller.status()['armed'])
        self.assertTrue(self.peer.closed)


if __name__=='__main__':unittest.main()
