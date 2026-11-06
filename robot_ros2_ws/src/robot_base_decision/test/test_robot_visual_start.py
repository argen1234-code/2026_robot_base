"""Launch the real entry script with fake components; never open hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class RobotVisualStartTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        source = Path(__file__).resolve().parents[4]
        for name in ('scripts', 'config', 'robot_ros2_ws/src/robot_base_decision'):
            shutil.copytree(source/name, self.root/name)
        (self.root/'robot_ros2_ws/install').mkdir()
        (self.root/'robot_ros2_ws/install/setup.bash').touch()
        (self.root/'depth/install').mkdir(parents=True)
        (self.root/'depth/install/setup.bash').touch()
        (self.root/'camera').touch()
        stub = self.root/'mamba_stub.py'
        stub.write_text('''#!/usr/bin/python3
from pathlib import Path
import signal, sys
name = Path(sys.argv[-1]).name
print({'color.sh': 'Astra color ready', 'depth.sh': 'Start device',
       'person.sh': 'person detector ready', 'visual.sh': 'visual follow ready',
       'chassis.sh': 'stub chassis'}[name], flush=True)
try:
    while True: signal.pause()
except KeyboardInterrupt:
    pass
''')
        stub.chmod(0o755)
        self.env = dict(os.environ, MAMBA_BIN=str(stub),
            ROBOT_STATE_DIR=str(self.root/'state'), ROBOT_LOG_DIR=str(self.root/'logs'),
            ASTRA_COLOR_DEVICE=str(self.root/'camera'), DEPTH_WS=str(self.root/'depth'))

    def command(self, *args):
        return subprocess.run(['bash', str(self.root/'scripts/robot.sh'), *args],
            env=self.env, text=True, capture_output=True, timeout=25)

    def tearDown(self):
        self.command('stop', '--force')
        self.temporary.cleanup()

    def test_preview_starts_only_camera_depth_person_and_visual(self):
        result = self.command('start', '--task', 'visual-approach', '--distance', '.2')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        manifest = (self.root/'state/manifest').read_text()
        self.assertEqual({line.split()[0] for line in manifest.splitlines()},
                         {'color', 'depth', 'person', 'visual'})
        log = (self.root/'logs/latest').resolve()
        script = (log/'visual.sh').read_text()
        self.assertNotIn('--execute', script)
        self.assertIn('--distance .2', script)
        self.assertIn('--depth-min-m "0.01"', (log/'person.sh').read_text())
        for generated in log.glob('*.sh'):
            subprocess.run(['bash', '-n', str(generated)], check=True)

    @unittest.skipUnless(Path('/dev/stm32').exists(), 'real entry preflight requires /dev/stm32')
    def test_motor_mode_starts_only_existing_bridge_and_direct_controller(self):
        result = self.command('start', '--task', 'visual-follow', '--motor')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        manifest = (self.root/'state/manifest').read_text()
        self.assertEqual({line.split()[0] for line in manifest.splitlines()},
                         {'color', 'depth', 'person', 'chassis', 'visual'})
        log = (self.root/'logs/latest').resolve()
        visual, chassis = (log/'visual.sh').read_text(), (log/'chassis.sh').read_text()
        self.assertIn('--execute', visual)
        self.assertIn('--follow', visual)
        self.assertIn('enabled:=true', chassis)
        self.assertIn('max_linear:=0.24', chassis)
        self.assertIn('max_angular:=0.55', chassis)
        self.assertIn('/visual_follow/unused_nav_velocity', chassis)

    def test_conflicting_nav_no_depth_and_unreachable_threshold_are_rejected(self):
        for extra in (['--nav', 'localize'], ['--no-depth'], ['--distance', '.005']):
            result = self.command('start', '--task', 'visual-follow', *extra)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((self.root/'state/manifest').exists())


if __name__ == '__main__':
    unittest.main()
