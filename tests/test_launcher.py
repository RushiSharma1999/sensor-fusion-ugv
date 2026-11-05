"""Test argument validation and the authoritative profile interface offline."""

from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class LauncherTests(unittest.TestCase):
    def test_invalid_options_fail_before_ros_or_hardware(self):
        for args in (['--moters'], ['--behavior', 'unknown'], ['--speed-profile', 'fast'],
                     ['--behavior', 'baseline', '--speed-profile', 'open_space'],
                     ['--behavior', 'enhanced', '--params', 'relative.yaml']):
            result = subprocess.run(['bash', str(ROOT / 'run_cloud36_test.sh'), *args],
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)

    def test_one_cloud36_command_path_and_shared_profile_selection(self):
        # The shell runner has no separate numeric controller/bridge ceilings.
        body = (ROOT / 'run_continuous_bot.sh').read_text()
        commands = [line for line in body.splitlines()
                    if line.startswith(('start vfh ', 'start bridge '))]
        self.assertEqual(len(commands), 2)
        for command in commands:
            self.assertIn('-p speed_profile:="$speed_profile"', command)
            self.assertNotIn('max_linear_speed', command)
            self.assertNotIn('forward_speed', command)
        self.assertIn('cloud_scan_num:=36', body)
        self.assertIn('exec bash "$root/run_continuous_bot.sh" "$@"',
                      (ROOT / 'run_cloud36_test.sh').read_text())


if __name__ == '__main__':
    unittest.main()
