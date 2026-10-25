"""Check summary-driven shutdown, timeout and batch scheduling with fake launches."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('batch', ROOT / 'run_ippo_8runs.py')
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)


class BatchTests(unittest.TestCase):
    def test_summary_stops_launch_and_detached_descendant(self):
        code = '''
import os,sys,time,json,subprocess
from pathlib import Path
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], start_new_session=True)
root=Path(os.environ['RESULT_ROOT']); root.mkdir(parents=True)
(root/'child_pid').write_text(str(child.pid))
time.sleep(.2)
(root/'summary.json').write_text(json.dumps({'type':'energy_constrained_mission_summary',
    'stop_reason':'coverage_target_reached','FCR':.95,'T95_sec':1.0}))
while True: time.sleep(1)
'''
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / 'run'
            result = batch.run_once([sys.executable, '-c', code], directory,
                                    timeout=5, poll_interval=.02, flush_seconds=.02,
                                    shutdown_timeout=.5)
            self.assertTrue(result['target_reached'])
            pid = int((directory / 'results/child_pid').read_text())
            self.assertNotIn(pid, batch.process_table())
            self.assertTrue((directory / 'console.log').exists())

    def test_missing_summary_times_out_and_cleans_up(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / 'run'
            code = 'import os,time; print(os.getpid(),flush=True); time.sleep(60)'
            with self.assertRaises(TimeoutError):
                batch.run_once([sys.executable, '-c', code], directory,
                               timeout=.2, poll_interval=.02, shutdown_timeout=.5)
            pid = int((directory / 'console.log').read_text().splitlines()[0])
            self.assertNotIn(pid, batch.process_table())

    def test_interrupt_cleans_up_current_launch(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / 'run'
            code = '''
import os,time
from pathlib import Path
root=Path(os.environ['RESULT_ROOT']);root.mkdir(parents=True)
(root/'pid').write_text(str(os.getpid()))
time.sleep(60)
'''
            def interrupt_when_started(root):
                if (root / 'pid').exists():
                    raise KeyboardInterrupt
                return None
            with patch.object(batch, 'find_summary', interrupt_when_started):
                with self.assertRaises(KeyboardInterrupt):
                    batch.run_once([sys.executable, '-c', code], directory,
                                   timeout=5, poll_interval=.02, shutdown_timeout=.5)
            pid = int((directory / 'results/pid').read_text())
            self.assertNotIn(pid, batch.process_table())

    def test_eight_runs_seven_pauses_and_portable_model(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'install').mkdir()
            (root / 'install/setup.bash').touch()
            model = root / batch.MODEL
            model.parent.mkdir(parents=True)
            model.touch()
            calls, pauses = [], []
            def run(command, directory, timeout):
                calls.append((list(command), directory))
                return {'stop_reason': 'coverage_target_reached', 'target_reached': True}
            with patch.object(batch, 'ROOT', root), patch.object(batch, 'run_once', run), \
                    patch.object(batch.time, 'sleep', pauses.append), \
                    patch.object(sys, 'argv', ['batch']):
                batch.main()
            self.assertEqual(len(calls), 8)
            self.assertEqual([command[3] for command, _ in calls], ['default'] * 4 + ['maze40'] * 4)
            for command, _ in calls:
                self.assertFalse(Path(command[4]).is_absolute())
                self.assertEqual(command[5:], [f'robot{i}_battery_capacity_ah:=0.40' for i in range(3)])
            self.assertEqual(len(set(directory for _, directory in calls)), 8)
            self.assertEqual(pauses, [30] * 7)
            manifest = next((root/'output').rglob('batch_summary.json'))
            data = json.loads(manifest.read_text())
            self.assertEqual(data['status'], 'completed')
            self.assertEqual(len(data['runs']), 8)

    def test_failed_launch_stops_batch(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'install').mkdir()
            (root / 'install/setup.bash').touch()
            model = root / batch.MODEL
            model.parent.mkdir(parents=True)
            model.touch()
            with patch.object(batch, 'ROOT', root), \
                    patch.object(batch, 'run_once', side_effect=RuntimeError('launch failed')) as run, \
                    patch.object(batch.time, 'sleep') as sleep, \
                    patch.object(sys, 'argv', ['batch']):
                with self.assertRaises(SystemExit) as error:
                    batch.main()
            self.assertEqual(error.exception.code, 1)
            self.assertEqual(run.call_count, 1)
            sleep.assert_not_called()
            manifest = next((root/'output').rglob('batch_summary.json'))
            self.assertEqual(json.loads(manifest.read_text())['status'], 'error')

    def test_inference_summary_is_not_mission_completion(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'summary.json').write_text(json.dumps({'type': 'mappo_inference_summary',
                                                       'stop_reason': 'node_shutdown'}))
            self.assertIsNone(batch.find_summary(root))
            (root/'summary.json').write_text('[]')
            self.assertIsNone(batch.find_summary(root))


if __name__ == '__main__':
    unittest.main()
