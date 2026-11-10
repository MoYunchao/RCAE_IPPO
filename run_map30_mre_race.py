#!/usr/bin/env python3
"""Run two additional Bandwidth-MRE trials and one original RACE trial on map30."""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

from run_ippo_8runs import find_summary, run_once, save_batch


ROOT = Path(__file__).resolve().parent
PROTOCOL = 'map30_mre_race_supplement_v1'
RACE_WS = Path('main_ablation_greedy/SenseSimulation_fix_v2/SenseSimulation_fix')
MRE_WS = Path('compare_with_rl/bandwidth_mre/SenseSimulation_fix')
RACE_MODEL = Path('models/inference/mappo/energy_distance_safe_seed42/models/best_model.pt')
MRE_MODEL = Path('models/inference/bandwidth_mre/sync_gain_topk_h1_80000/models/best_model.pt')


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def default_experiment_root():
    candidates = []
    if os.environ.get('RACE_EXPERIMENT_ROOT'):
        return Path(os.environ['RACE_EXPERIMENT_ROOT']).expanduser()
    candidates.extend([Path.home() / 'energy_mission', ROOT.parent / 'energy_mission',
                       Path.home() / 'exploration/fuzzy_total/v2_compare/energy_mission'])
    return next((p for p in candidates if (p / RACE_WS).is_dir() and (p / MRE_WS).is_dir()),
                candidates[0])


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment-root', type=Path, default=default_experiment_root(),
                        help='Directory containing main_ablation_greedy/ and compare_with_rl/')
    parser.add_argument('--mre-workspace', type=Path)
    parser.add_argument('--race-workspace', type=Path)
    parser.add_argument('--mre-model', type=Path, help='Explicit model override; relative to MRE workspace')
    parser.add_argument('--race-model', type=Path, help='Explicit model override; relative to RACE workspace')
    parser.add_argument('--mre-runs', type=int, default=2)
    parser.add_argument('--race-runs', type=int, default=1)
    parser.add_argument('--timeout-sec', type=float, default=1800,
                        help='Wrapper watchdog, not the mission time limit')
    parser.add_argument('--pause-sec', type=float, default=30)
    parser.add_argument('--output-root', type=Path, default=ROOT / 'output/map30_mre_race_batches')
    parser.add_argument('--resume-dir', type=Path,
                        help='Resume this batch without repeating its completed mission summaries')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    if min(args.mre_runs, args.race_runs) < 0 or args.mre_runs + args.race_runs == 0:
        parser.error('run counts must be nonnegative, with at least one trial')
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0:
        parser.error('--timeout-sec must be positive and finite')
    if not math.isfinite(args.pause_sec) or args.pause_sec < 0:
        parser.error('--pause-sec must be nonnegative and finite')
    return args


def build_plan(args):
    base = args.experiment_root.expanduser().resolve()
    jobs = []
    for method, count, workspace, relative_ws, override, default_model, launcher in [
        ('bandwidth_mre', args.mre_runs, args.mre_workspace, MRE_WS, args.mre_model,
         MRE_MODEL, 'start_all_bandwidth_mre.sh'),
        ('race', args.race_runs, args.race_workspace, RACE_WS, args.race_model,
         RACE_MODEL, 'start_all_mappo.sh'),
    ]:
        if not count:
            continue
        work = (workspace.expanduser() if workspace else base / relative_ws).resolve()
        model_arg = override.expanduser() if override else default_model
        model = (model_arg if model_arg.is_absolute() else work / model_arg).resolve()
        if not (work / launcher).is_file():
            raise FileNotFoundError(f'Missing launcher: {work / launcher}')
        if not model.is_file():
            raise FileNotFoundError(f'Missing {method} model: {model}; specify its workspace/model explicitly')
        command = ['bash', launcher, '--map', 'default', str(model)]
        command += [f'robot{i}_battery_capacity_ah:=0.40' for i in range(3)]
        proof = sha256(model)
        for trial in range(1, count + 1):
            jobs.append({'id': f'{method}_{trial:02d}', 'method': method, 'map': 'default',
                         'workspace': str(work), 'model': str(model), 'model_sha256': proof,
                         'command': command.copy(), 'battery_capacity_ah': [0.4] * 3})
    return jobs


def validate_summary(summary):
    if summary.get('type') != 'energy_constrained_mission_summary':
        raise ValueError('Not a mission-level summary')
    if int(summary.get('known_cells', 0)) <= 0:
        raise ValueError('No mapped cells: startup failure; batch stopped, artifacts retained')
    for key in ['FCR', 'ATL_m', 'NFR', 'runtime_sec']:
        value = summary.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f'Invalid mission metric {key}: {value!r}')
    if not 0 <= summary['FCR'] <= 1 or summary['ATL_m'] < 0 or summary['runtime_sec'] < 0:
        raise ValueError('Mission metrics outside expected range')
    if summary['NFR'] not in (0, 1, 2, 3):
        raise ValueError('NFR must be an integer count from 0 to 3')
    if summary.get('stop_reason') not in ('coverage_target_reached', 'max_mission_time',
                                          'all_robots_failed', 'no_reachable_frontier'):
        raise ValueError(f'Unexpected mission stop: {summary.get("stop_reason")}')


def completed_record(directory):
    found = find_summary(directory / 'results')
    if found is None:
        raise RuntimeError(f'No mission summary in {directory}')
    path, summary = found
    validate_summary(summary)
    return {'summary_path': str(path.relative_to(directory)), 'summary_sha256': sha256(path),
            **{key: summary.get(key) for key in
               ['stop_reason', 'runtime_sec', 'FCR', 'T95_sec', 'ATL_m', 'NFR', 'known_cells']}}


def ensure_idle():
    """Avoid launching on top of an existing Gazebo/Nav2 experiment; never kill it."""
    busy = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = path.read_bytes().split(b'\0')
        except OSError:
            continue
        executable = Path(os.fsdecode(argv[0])).name if argv and argv[0] else ''
        if executable in {'gzserver', 'gzclient', 'controller_server', 'bt_navigator',
                          'exploration_supervisor.py'}:
            busy.append(f'{path.parent.name}:{executable}')
    if busy:
        raise RuntimeError('An experiment is already running: ' + ', '.join(busy))


def run_batch(args, jobs, directory):
    manifest_path = directory / 'batch_summary.json'
    if args.resume_dir:
        manifest = json.loads(manifest_path.read_text())
        if manifest.get('protocol') != PROTOCOL or manifest.get('plan') != jobs:
            raise ValueError('Resume plan/workspaces/models differ from the recorded batch')
        if manifest.get('timeout_sec') != args.timeout_sec:
            raise ValueError('Use the original --timeout-sec when resuming')
    else:
        directory.mkdir(parents=True, exist_ok=False)
        manifest = {'protocol': PROTOCOL, 'created_at': datetime.now().isoformat(),
                    'plan': jobs, 'timeout_sec': args.timeout_sec, 'runs': [], 'status': 'running'}
    manifest['status'] = 'running'
    save_batch(manifest_path, manifest)
    launched = False
    try:
        for job in jobs:
            # Recover a mission completed before an interruption during artifact flushing.
            existing = [r for r in manifest['runs'] if r['job_id'] == job['id']]
            complete = next((r for r in existing if r['status'] == 'completed'), None)
            if complete:
                path = directory / complete['relative_directory']
                record = completed_record(path)
                if record['summary_sha256'] != complete['summary_sha256']:
                    raise ValueError(f'Completed summary changed: {path}')
                continue
            recovered = False
            for row in existing:
                if row['status'] == 'completed':
                    continue
                path = directory / row['relative_directory']
                if find_summary(path / 'results'):
                    try:
                        record = completed_record(path)
                    except ValueError:
                        continue  # Preserve invalid startup artifacts and create a fresh attempt.
                    row.update(record, status='completed')
                    save_batch(path / 'model_provenance.json',
                               {k: job[k] for k in ['method', 'model', 'model_sha256', 'command', 'workspace']})
                    save_batch(manifest_path, manifest)
                    recovered = True
                    break
            if recovered:
                continue
            if launched or args.resume_dir:
                time.sleep(args.pause_sec)
            ensure_idle()
            if sha256(Path(job['model'])) != job['model_sha256']:
                raise ValueError('Model file changed during the batch')
            attempt = directory / job['id'] / f'attempt_{len(existing) + 1:02d}'
            row = {'job_id': job['id'], 'status': 'running',
                   'relative_directory': str(attempt.relative_to(directory))}
            manifest['runs'].append(row)
            save_batch(manifest_path, manifest)
            launched = True
            print(f'Running {job["id"]}', flush=True)
            try:
                run_once(job['command'], attempt, timeout=args.timeout_sec, cwd=Path(job['workspace']))
                row.update(completed_record(attempt), status='completed')
                save_batch(attempt / 'model_provenance.json',
                           {k: job[k] for k in ['method', 'model', 'model_sha256', 'command', 'workspace']})
            except BaseException as exc:
                row.update(status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'error',
                           error=str(exc))
                raise
            finally:
                save_batch(manifest_path, manifest)
        manifest['status'] = 'completed'
    except BaseException as exc:
        manifest.update(status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'error',
                        error=str(exc))
        raise
    finally:
        save_batch(manifest_path, manifest)
    return manifest


def main(argv=None):
    args = parse_args(argv)
    jobs = build_plan(args)
    for job in jobs:
        print(f'{job["id"]}: cwd={job["workspace"]}\n  {json.dumps(job["command"])}', flush=True)
    if args.dry_run:
        print('Dry run only. Actual execution also requires built install/setup.bash in both workspaces.')
        return
    for job in jobs:
        if not (Path(job['workspace']) / 'install/setup.bash').is_file():
            raise FileNotFoundError(f'Build/source the experiment workspace first: {job["workspace"]}')
    ensure_idle()
    directory = (args.resume_dir.expanduser().resolve() if args.resume_dir else
                 args.output_root.expanduser().resolve() / datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    # Share the IPPO batch lock so these entry points cannot overlap in this checkout.
    (ROOT / 'output').mkdir(exist_ok=True)
    with (ROOT / 'output/ippo_supplement.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        print(f'Batch directory: {directory}', flush=True)
        run_batch(args, jobs, directory)
    print(f'Completed. Results: {directory}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted; current launch cleaned up. Use --resume-dir to continue.', file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f'Batch stopped: {exc}', file=sys.stderr)
        sys.exit(1)
