import importlib.util
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('prepare',ROOT/'scripts/esp32_prepare_lidar.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class FakeSerial:
    def __init__(self,latched=1,arm_ok=True,hardware_ok=1):
        self.latched=latched;self.arm_ok=arm_ok;self.hardware_ok=hardware_ok
        self.writes=[];self.pending=[]
    def write(self,payload):
        self.writes.append(payload)
        if payload==b'DEBUG 0\n':self.pending.append(b'ACK DEBUG 0\n')
        if payload==b'STATUS\n':self.pending.append(
            f'STATE host_active 0 stop_latched {self.latched} ok {self.hardware_ok} reason test pwmL 0 pwmR 0 sonar 1\n'.encode())
        if payload==b'ARM\n' and self.arm_ok:
            self.latched=0;self.pending.append(b'ACK ARM stopped\n')
        if payload==b'STOP\n':self.latched=1
        return len(payload)
    def readline(self):return self.pending.pop(0) if self.pending else b''

class PrepareTests(unittest.TestCase):
    def test_latch_not_silently_released(self):
        s=FakeSerial()
        with self.assertRaisesRegex(m.PrepareError,'STOP IS LATCHED'):m.prepare(s,timeout=.01)
        self.assertNotIn(b'ARM\n',s.writes);self.assertEqual(s.latched,1)
    def test_explicit_arm_only_after_zero(self):
        s=FakeSerial();r=m.prepare(s,arm=True,timeout=.01)
        self.assertEqual(r['before']['stop_latched'],1);self.assertEqual(r['after']['stop_latched'],0)
        self.assertEqual(s.writes[0],m.ZERO);self.assertEqual(s.writes[-1],m.ZERO)
        self.assertTrue(all(p in (m.ZERO,b'DEBUG 0\n',b'STATUS\n',b'ARM\n') for p in s.writes))
    def test_ready_firmware_does_not_need_arm(self):
        s=FakeSerial(latched=0);r=m.prepare(s,timeout=.01)
        self.assertFalse(r['arm_sent']);self.assertNotIn(b'ARM\n',s.writes)
    def test_arm_timeout_relatches(self):
        s=FakeSerial(arm_ok=False)
        with self.assertRaises(m.PrepareError):m.prepare(s,arm=True,timeout=.001)
        self.assertIn(b'STOP\n',s.writes);self.assertEqual(s.writes[-1],m.ZERO)
    def test_hardware_fault_never_arms(self):
        s=FakeSerial(hardware_ok=0)
        with self.assertRaises(m.PrepareError):m.prepare(s,arm=True,timeout=.01)
        self.assertNotIn(b'ARM\n',s.writes)
    def test_old_or_malformed_state_rejected(self):
        for line in ('STATE mode 1 pwmL 1 pwmR 2 ok 1','STATE host_active nan',
                     'STATE host_active 0 stop_latched 0 ok 1 pwmL 0 pwmR 0 pwmL 10'):
            self.assertIsNone(m.parse_state(line))
    def test_state_after_marker_ignores_stale_buffered_motion(self):
        s=FakeSerial(latched=0)
        s.pending.append(b'STATE host_active 1 stop_latched 0 ok 1 reason old pwmL 100 pwmR 100\n')
        r=m.prepare(s,timeout=.01);self.assertEqual(r['after']['pwmL'],0)
    def test_no_response_leaves_zero(self):
        s=FakeSerial();s.readline=lambda:b''
        with self.assertRaises(m.PrepareError):m.prepare(s,timeout=.001)
        self.assertEqual(s.writes[-1],m.ZERO)


class FakeV4Serial:
    def __init__(self, mode='IDLE', pwm='0/0', doing='STOPPED (IDLE)', auto=0, wrong=False):
        self.writes=[];self.pending=[]
        self.mode=mode;self.pwm=pwm;self.doing=doing;self.auto=auto;self.wrong=wrong
    def write(self,payload):
        self.writes.append(payload)
        if payload==b'STOP\n':self.pending.append(b'ACK STOP\n')
        if payload==b'HELP\n':self.pending.append(
            b'CMD v <m/s> w <rad/s> | STOP | ARM | STATUS\n' if self.wrong else
            b'CMD v <m/s> w <rad/s> | PWM <L> <R> | STOP | ZERO | STATUS | MODE AUTO | DEBUG 0/1\n')
        if payload==b'DEBUG 1\n':
            self.pending += [b'ACK DEBUG 1\n',
                (f'DBG|T:200|MODE:{self.mode}|DOING:{self.doing}|WHY:|F:noecho|L:noecho|R:noecho|'
                 f'V:0.00|W:0.00|PWM:{self.pwm}|AUTOEN:{self.auto}|ENC:0/0\n').encode()]
        return len(payload)
    def readline(self):return self.pending.pop(0) if self.pending else b''

class OriginalV4PrepareTests(unittest.TestCase):
    def test_original_default_telemetry_needs_no_state_or_arm(self):
        s=FakeV4Serial();out=m.prepare_v4(s,timeout=.01)
        self.assertEqual(out['firmware'],'v4');self.assertFalse(out['arm_sent'])
        self.assertEqual(out['stopped']['mode'],'IDLE')
        self.assertEqual(s.writes[-1],m.ZERO)
        self.assertTrue(all(p in (b'STOP\n',m.ZERO,b'HELP\n',b'DEBUG 1\n') for p in s.writes))
    def test_wrong_firmware_rejected_and_stopped(self):
        s=FakeV4Serial(wrong=True)
        with self.assertRaisesRegex(m.PrepareError,'Expected original'):m.prepare_v4(s,timeout=.01)
        self.assertEqual(s.writes[-2:],[b'STOP\n',m.ZERO])
    def test_hardware_fault_rejected(self):
        s=FakeV4Serial(doing='HARDWARE ERROR')
        with self.assertRaisesRegex(m.PrepareError,'hardware'):m.prepare_v4(s,timeout=.01)
    def test_nonzero_or_auto_state_rejected(self):
        for kwargs in (dict(mode='AUTO'),dict(auto=1),dict(pwm='8/0')):
            s=FakeV4Serial(**kwargs)
            with self.assertRaisesRegex(m.PrepareError,'did not confirm'):m.prepare_v4(s,timeout=.01)
            self.assertEqual(s.writes[-1],m.ZERO)
    def test_stale_debug_before_ack_is_ignored(self):
        s=FakeV4Serial();s.pending=[b'DBG|T:100|MODE:AUTO|DOING:MOVING|PWM:100/100|AUTOEN:1\n']
        self.assertEqual(m.prepare_v4(s,timeout=.01)['stopped']['mode'],'IDLE')
    def test_safe_identity_retry_after_reset(self):
        clock_value=[0.0]
        def clock():
            clock_value[0] += .05
            return clock_value[0]
        s=FakeV4Serial()
        result=m.wait_for(s,lambda line: True if 'MODE AUTO' in line else None,
                          3,clock=clock,retry=lambda:s.write(b'HELP\n'))
        self.assertTrue(result)
        self.assertEqual(s.writes,[b'HELP\n'])
    def test_v4_no_response_leaves_stop_and_zero(self):
        s=FakeV4Serial();s.readline=lambda:b''
        with self.assertRaises(m.PrepareError):m.prepare_v4(s,timeout=.001)
        self.assertEqual(s.writes[-2:],[b'STOP\n',m.ZERO])

if __name__=='__main__':unittest.main(verbosity=2)
