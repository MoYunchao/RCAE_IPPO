"""Exercise the supplement schedule and recovery without ROS or Gazebo."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('supplement', ROOT / 'run_map30_mre_race.py')
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)


def setup(root):
    for ws, launcher, model in [(batch.MRE_WS, 'start_all_bandwidth_mre.sh', batch.MRE_MODEL),
                                 (batch.RACE_WS, 'start_all_mappo.sh', batch.RACE_MODEL)]:
        work = root / ws
        work.mkdir(parents=True)
        (work / launcher).write_text('# fake launcher\n')
        (work / model).parent.mkdir(parents=True)
        (work / model).write_bytes(b'original policy')
    return batch.parse_args(['--experiment-root', str(root), '--pause-sec', '0'])


def fake_run(command, directory, timeout, cwd):
    path = directory / 'results/map_default/test/trial/summary.json'
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({'type': 'energy_constrained_mission_summary',
                               'known_cells': 100, 'FCR': .8, 'ATL_m': 200,
                               'runtime_sec': 600, 'NFR': 2, 'T95_sec': None,
                               'stop_reason': 'max_mission_time'}))


class SupplementTests(unittest.TestCase):
    def test_plan_uses_historical_models_and_matched_capacity(self):
        with tempfile.TemporaryDirectory() as temp:
            args = setup(Path(temp))
            jobs = batch.build_plan(args)
            self.assertEqual([j['method'] for j in jobs], ['bandwidth_mre', 'bandwidth_mre', 'race'])
            for j in jobs:
                self.assertEqual(j['command'][2:4], ['--map', 'default'])
                self.assertEqual(j['command'][5:], [f'robot{i}_battery_capacity_ah:=0.40' for i in range(3)])
            self.assertIn('energy_distance_safe_seed42', jobs[-1]['model'])
            self.assertNotIn('mappo_v3', jobs[-1]['model'])

    def test_mission_failures_count_and_completed_batch_is_not_repeated(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            args = setup(root)
            jobs = batch.build_plan(args)
            directory = root / 'batch'
            with patch.object(batch, 'run_once', side_effect=fake_run) as run, \
                    patch.object(batch, 'ensure_idle'), patch.object(batch.time, 'sleep') as sleep:
                result = batch.run_batch(args, jobs, directory)
                self.assertEqual(run.call_count, 3)
                self.assertEqual(sleep.call_count, 2)
                self.assertEqual(result['status'], 'completed')
                self.assertTrue(all(r['NFR'] == 2 for r in result['runs']))
                args.resume_dir = directory
                batch.run_batch(args, jobs, directory)
                self.assertEqual(run.call_count, 3)

    def test_launch_error_stops_later_trials(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            args = setup(root)
            with patch.object(batch, 'run_once', side_effect=RuntimeError('launch failed')) as run, \
                    patch.object(batch, 'ensure_idle'):
                with self.assertRaisesRegex(RuntimeError, 'launch failed'):
                    batch.run_batch(args, batch.build_plan(args), root / 'batch')
                self.assertEqual(run.call_count, 1)
            result = json.loads((root / 'batch/batch_summary.json').read_text())
            self.assertEqual(result['status'], 'error')

    def test_interruption_after_summary_is_recovered_without_rerunning(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            args = setup(root)
            jobs = batch.build_plan(args)
            def interrupted(*a, **kw):
                fake_run(*a, **kw)
                raise KeyboardInterrupt
            with patch.object(batch, 'run_once', side_effect=interrupted), patch.object(batch, 'ensure_idle'):
                with self.assertRaises(KeyboardInterrupt):
                    batch.run_batch(args, jobs, root / 'batch')
            args.resume_dir = root / 'batch'
            with patch.object(batch, 'run_once', side_effect=fake_run) as run, \
                    patch.object(batch, 'ensure_idle'), patch.object(batch.time, 'sleep'):
                result = batch.run_batch(args, jobs, args.resume_dir)
                self.assertEqual(run.call_count, 2)
                self.assertEqual(result['status'], 'completed')

    def test_zero_map_startup_does_not_count_and_resume_uses_new_attempt(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            args = setup(root)
            jobs = batch.build_plan(args)
            def empty(*a, **kw):
                fake_run(*a, **kw)
                p = next(a[1].rglob('summary.json'))
                d = json.loads(p.read_text())
                d['known_cells'] = 0
                p.write_text(json.dumps(d))
            with patch.object(batch, 'run_once', side_effect=empty), patch.object(batch, 'ensure_idle'):
                with self.assertRaisesRegex(ValueError, 'No mapped cells'):
                    batch.run_batch(args, jobs, root / 'batch')
            args.resume_dir = root / 'batch'
            with patch.object(batch, 'run_once', side_effect=fake_run), \
                    patch.object(batch, 'ensure_idle'), patch.object(batch.time, 'sleep'):
                result = batch.run_batch(args, jobs, args.resume_dir)
            self.assertEqual(sum(r['status'] == 'completed' for r in result['runs']), 3)
            self.assertEqual(result['runs'][0]['status'], 'error')
            self.assertIn('attempt_02', result['runs'][1]['relative_directory'])

    def test_changed_model_rejects_resume(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            args = setup(root)
            jobs = batch.build_plan(args)
            with patch.object(batch, 'run_once', side_effect=fake_run), \
                    patch.object(batch, 'ensure_idle'), patch.object(batch.time, 'sleep'):
                batch.run_batch(args, jobs, root / 'batch')
            Path(jobs[0]['model']).write_bytes(b'different model')
            args.resume_dir = root / 'batch'
            with self.assertRaisesRegex(ValueError, 'Resume plan'):
                batch.run_batch(args, batch.build_plan(args), args.resume_dir)

    def test_explicit_workspaces_and_arbitrary_counts(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            setup(root)
            args = batch.parse_args(['--mre-workspace', str(root / batch.MRE_WS),
                                     '--race-workspace', str(root / batch.RACE_WS),
                                     '--mre-runs', '0', '--race-runs', '2'])
            self.assertEqual([j['method'] for j in batch.build_plan(args)], ['race', 'race'])


if __name__ == '__main__':
    unittest.main()
