import pathlib, subprocess, unittest, os
SCRIPT=pathlib.Path(os.environ.get('LIDAR_LAUNCHER', str(pathlib.Path(__file__).resolve().parents[2]/'start_lidar_only.sh')))
class LauncherTests(unittest.TestCase):
    def run_cli(self,*args):
        return subprocess.run(['bash',str(SCRIPT),*args],capture_output=True,text=True)
    def test_plain_start_enables_bridge_with_v4_handshake(self):
        r=self.run_cli('--dry-run');self.assertEqual(r.returncode,0,r.stderr)
        self.assertIn('ros2 run esp_bridge cmd_vel_to_serial',r.stdout)
        self.assertIn('--firmware v4',r.stdout)
        self.assertNotIn('--arm',r.stdout)
    def test_stationary_keeps_bridge_disabled(self):
        r=self.run_cli('--stationary','--dry-run')
        self.assertEqual(r.returncode,0,r.stderr)
        self.assertIn('6 MOTOR BRIDGE: DISABLED',r.stdout)
        self.assertNotIn('ros2 run esp_bridge',r.stdout)
    def test_drive_includes_bounded_profile_and_serial_identity(self):
        r=self.run_cli('--dry-run','--drive');self.assertEqual(r.returncode,0,r.stderr)
        self.assertIn('ros2 run esp_bridge cmd_vel_to_serial',r.stdout)
        self.assertIn('lidar_only.yaml',r.stdout)
        self.assertIn('usb-1a86_USB_Single_Serial_5A84021142-if00',r.stdout)
    def test_exactly_six_panes_and_no_unwanted_nodes(self):
        r=self.run_cli('--drive','--dry-run')
        self.assertEqual(len(r.stdout.splitlines()),6)
        for bad in ('camera_follow_node','vision_uart_receiver_node','fusion_node','gap_follow','foxglove'):
            self.assertNotIn(bad,r.stdout)
    def test_custom_port_is_one_quoted_argument(self):
        r=self.run_cli('--drive','--dry-run','--port','/tmp/a b;echo INJECTED')
        self.assertIn('port:=/tmp/a\\ b\\;echo\\ INJECTED',r.stdout)
        self.assertEqual(r.returncode,0,r.stderr)
    def test_arm_rejected_for_original_v4(self):
        for args in (('--stationary','--arm'), ('--drive','--arm')):
            self.assertEqual(self.run_cli(*args,'--dry-run').returncode,2)
        r=self.run_cli('--drive','--dry-run')
        self.assertIn('--firmware v4',r.stdout)
        self.assertNotIn('--arm',r.stdout)
        self.assertNotIn('MODE AUTO',r.stdout)
        self.assertNotIn('PWM ',r.stdout)
    def test_bad_flag_rejected(self): self.assertEqual(self.run_cli('--wrong').returncode,2)
    def test_missing_port_rejected(self): self.assertEqual(self.run_cli('--port').returncode,2)
    def test_help_needs_no_ros(self): self.assertEqual(self.run_cli('--help').returncode,0)
if __name__=='__main__': unittest.main(verbosity=2)
