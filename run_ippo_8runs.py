#!/usr/bin/env python3
"""Supplement existing IPPO results to five usable trials per Maze map."""
import argparse
import fcntl
import hashlib
from datetime import datetime
from contextlib import nullcontext
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parent
MODEL = 'models/inference/ippo/best_model.pt'
BASELINE = 'configs/ippo_existing_results_20261025.json'
PROTOCOL = 'supplement_to_five_v1'


def command_for(map_name, battery_capacity='0.40'):
    return ['bash', 'start_all_ippo.sh', '--map', map_name, MODEL,
            f'robot0_battery_capacity_ah:={battery_capacity}', f'robot1_battery_capacity_ah:={battery_capacity}',
            f'robot2_battery_capacity_ah:={battery_capacity}']


def process_table():
    """Linux process identities: pid -> (start ticks, parent pid, process group)."""
    table = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if fields[0] != 'Z':
                table[int(path.parent.name)] = (fields[19], int(fields[1]), int(fields[2]))
        except (OSError, ValueError, IndexError):
            continue
    return table


class Experiment:
    """Track only this launch and its descendants, including child process groups."""
    def __init__(self, command, result_root, log_path, cwd=ROOT):
        self.log = log_path.open('w')
        self.process = None
        try:
            self.process = subprocess.Popen(
                command, cwd=cwd, env={**os.environ, 'RESULT_ROOT': str(result_root)},
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True,
            )
        except BaseException:
            self.log.close()
            raise
        self.identities = {}
        table = process_table()
        if self.process.pid in table:
            self.identities[self.process.pid] = table[self.process.pid][0]
        self.refresh()

    def refresh(self):
        self.process.poll()  # Reap the launch wrapper if it has exited.
        table = process_table()
        live = {pid for pid, start in self.identities.items()
                if pid in table and table[pid][0] == start}
        # The leader may have exited, while its pipeline is still in its group.
        live.update(pid for pid, row in table.items() if row[2] == self.process.pid)
        while True:
            children = {pid for pid, row in table.items() if row[1] in live}
            if children <= live:
                break
            live.update(children)
        for pid in live:
            self.identities[pid] = table[pid][0]
        return {pid: table[pid] for pid in live}

    def stop(self, timeout=20.0):
        try:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                live = self.refresh()
                if not live:
                    return
                if any(row[2] == self.process.pid for row in live.values()):
                    try:
                        os.killpg(self.process.pid, sig)
                    except ProcessLookupError:
                        pass
                # Some launched children create their own process groups.
                for pid, row in live.items():
                    if row[2] != self.process.pid:
                        try:
                            os.kill(pid, sig)
                        except ProcessLookupError:
                            pass
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if not self.refresh():
                        return
                    time.sleep(0.1)
            if self.refresh():
                raise RuntimeError('Some experiment processes remain; refusing to start another run')
        finally:
            self.log.close()


def find_summary(result_root):
    for path in result_root.rglob('summary.json'):
        try:
            summary = json.loads(path.read_text())
        except (OSError, ValueError):
            continue  # The supervisor may still be writing the file.
        if (isinstance(summary, dict) and summary.get('type') == 'energy_constrained_mission_summary'
                and summary.get('stop_reason')):
            return path, summary
    return None


def run_once(command, directory, timeout=1800.0, poll_interval=1.0,
             flush_seconds=3.0, shutdown_timeout=20.0, cwd=ROOT):
    directory.mkdir(parents=True, exist_ok=False)
    result_root = directory / 'results'
    log_path = directory / 'console.log'
    experiment = Experiment(command, result_root, log_path, cwd)
    start = time.monotonic()
    try:
        while True:
            experiment.refresh()
            completed = find_summary(result_root)
            if completed:
                path, summary = completed
                # finish() writes final CSV rows and publishes mission stop after
                # writing summary.json; allow those and inference logs to flush.
                time.sleep(flush_seconds)
                return {
                    'summary_path': str(path), 'stop_reason': summary['stop_reason'],
                    'target_reached': summary['stop_reason'] == 'coverage_target_reached',
                    'T95_sec': summary.get('T95_sec'), 'FCR': summary.get('FCR'),
                    'known_cells': summary.get('known_cells', 0),
                    'runtime_sec': summary.get('runtime_sec'),
                    'ATL_m': summary.get('ATL_m'), 'NFR': summary.get('NFR'),
                    'summary_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                }
            if experiment.process.poll() is not None:
                raise RuntimeError(f'Launch exited without a mission summary; see {log_path}')
            if time.monotonic() - start >= timeout:
                raise TimeoutError(f'No mission summary within {timeout:g}s; see {log_path}')
            time.sleep(poll_interval)
    finally:
        experiment.stop(shutdown_timeout)


def save_batch(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def existing_trials(root, maps, capacity, baseline):
    """Use the frozen historical ledger plus completed supplemental attempts.

    Do not choose trials by FCR/reward. A known, nonempty map is the only
    startup eligibility gate; depleted robots and other mission outcomes count.
    """
    records = {name: [] for name in maps}
    seen = set()
    for row in baseline['records']:
        if row['map'] in records and row['counted']:
            if row['capacity_ah'] != [float(capacity)] * 3:
                raise ValueError('Historical battery capacity differs')
            records[row['map']].append(row)
            seen.add(row['summary_sha256'])
    for manifest_path in sorted((root / 'output').rglob('batch_summary.json')):
        data = json.loads(manifest_path.read_text())
        if data.get('protocol') != PROTOCOL:
            continue
        if data.get('checkpoint_sha256') != baseline['checkpoint_sha256']:
            raise ValueError(f'Supplement model differs: {manifest_path}')
        for row in data['runs']:
            if row.get('map') not in records:
                continue
            if data['battery_capacity_ah'] != [float(capacity)] * 3:
                raise ValueError(f'Supplement battery capacity differs: {manifest_path}')
            if row.get('relative_summary_path'):
                path = manifest_path.parent / row['relative_summary_path']
                summary = json.loads(path.read_text())
            else:
                # Recover a completed mission if the wrapper was interrupted
                # during artifact flushing or process cleanup.
                completed = find_summary(manifest_path.parent / row['relative_directory'] / 'results')
                if completed is None:
                    continue
                path, summary = completed
            if int(summary.get('known_cells', 0)) <= 0:
                continue
            proof = json.loads((path.parent / 'checkpoint_provenance.json').read_text())
            if proof['sha256'] != baseline['checkpoint_sha256']:
                raise ValueError(f'Supplement checkpoint differs: {path}')
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if row.get('summary_sha256') and digest != row['summary_sha256']:
                raise ValueError(f'Summary was changed: {path}')
            if digest not in seen:
                records[row['map']].append({**row, 'summary_sha256': digest,
                    'recovered_summary': str(path), 'FCR': summary['FCR']})
                seen.add(digest)
    return records


def main(*, maps=('default', 'maze40'), battery_capacity='0.40',
         batch_folder='ippo_supplement_batches', description=__doc__):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--timeout-sec', type=float, default=1800)
    parser.add_argument('--max-startup-attempts', type=int, default=3,
                        help='Maximum attempts per missing sample; zero-map attempts never count')
    parser.add_argument('--dry-run', action='store_true', help='Show missing trials without launching ROS')
    args = parser.parse_args()
    if not 0 < args.timeout_sec < float('inf') or args.max_startup_attempts < 1:
        parser.error('timeout must be positive and finite; attempts must be positive')
    baseline = json.loads((ROOT / BASELINE).read_text())
    target = int(baseline['target_per_map'])
    model_hash = hashlib.sha256((ROOT / MODEL).read_bytes()).hexdigest()
    if model_hash != baseline['checkpoint_sha256']:
        parser.error('Checkpoint differs from existing trials; use the same model for supplementation')

    # Keep both map scripts from launching overlapping Gazebo experiments.
    if not args.dry_run:
        (ROOT / 'output').mkdir(exist_ok=True)
    lock_context = nullcontext() if args.dry_run else (ROOT / 'output/ippo_supplement.lock').open('a')
    with lock_context as lock:
        if not args.dry_run:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                parser.error('Another supplemental batch is running; run these scripts sequentially')
        records = existing_trials(ROOT, maps, battery_capacity, baseline)
        plan = []
        for name in maps:
            count = len(records[name])
            print(f'{name}: existing={count}, target={target}, missing={max(0, target-count)}, '
                  f'capacity={battery_capacity} Ah per robot', flush=True)
            if count > target:
                parser.error(f'{name} already has more than {target} eligible trials; inspect the ledger')
            plan.extend((name, repeat) for repeat in range(count + 1, target + 1))
        for index, (name, repeat) in enumerate(plan, 1):
            print(f'[{index}/{len(plan)}] sample {repeat}: ' + ' '.join(command_for(name, battery_capacity)), flush=True)
        if args.dry_run or not plan:
            return
        if not (ROOT / 'install/setup.bash').is_file():
            parser.error('Build this workspace first: colcon build --symlink-install')
        execute_plan(plan, records, baseline, battery_capacity, batch_folder, args)


def execute_plan(plan, records, baseline, battery_capacity, batch_folder, args):
    def on_terminate(signum, frame):
        raise KeyboardInterrupt
    previous_handler = signal.signal(signal.SIGTERM, on_terminate)
    batch_dir = ROOT / 'output' / batch_folder / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    batch_dir.mkdir(parents=True)
    manifest = {'protocol': PROTOCOL, 'model': MODEL,
                'checkpoint_sha256': baseline['checkpoint_sha256'],
                'baseline_manifest': BASELINE,
                'baseline_manifest_sha256': hashlib.sha256((ROOT / BASELINE).read_bytes()).hexdigest(),
                'source_archive_sha256': baseline.get('source_archive_sha256'),
                'target_per_map': baseline['target_per_map'],
                'existing_counts': {name: len(rows) for name, rows in records.items()},
                'existing_trials': json.loads(json.dumps(records)),
                'pause_seconds': 30, 'battery_capacity_ah': [float(battery_capacity)] * 3,
                'timeout_sec': args.timeout_sec, 'status': 'running', 'runs': []}
    manifest_path = batch_dir / 'batch_summary.json'
    save_batch(manifest_path, manifest)
    print('Batch results:', batch_dir, flush=True)
    try:
        for index, (name, repeat) in enumerate(plan, 1):
            for attempt in range(1, args.max_startup_attempts + 1):
                command = command_for(name, battery_capacity)
                relative = Path(name) / f'sample_{repeat:02d}' / f'attempt_{attempt:02d}'
                directory = batch_dir / relative
                row = {'map': name, 'sample_index': repeat, 'attempt': attempt,
                       'command': command, 'relative_directory': str(relative),
                       'status': 'running', 'counted': False}
                manifest['runs'].append(row)
                save_batch(manifest_path, manifest)
                print(f'[{index}/{len(plan)}] {name} sample={repeat} attempt={attempt}; log: {directory / "console.log"}', flush=True)
                try:
                    row.update(run_once(command, directory, args.timeout_sec))
                    row['relative_summary_path'] = str(Path(row['summary_path']).relative_to(batch_dir))
                    proof = json.loads((Path(row['summary_path']).parent / 'checkpoint_provenance.json').read_text())
                    if proof['sha256'] != baseline['checkpoint_sha256']:
                        raise ValueError('Run checkpoint differs from the frozen baseline')
                    if int(row['known_cells']) > 0:
                        row.update(status='completed', counted=True)
                        records[name].append(dict(row))
                    else:
                        row.update(status='startup_failed', exclusion_reason='zero known map cells; retain raw result and retry')
                except BaseException as exc:
                    row.update(status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'error',
                               error=str(exc) or type(exc).__name__)
                    raise
                finally:
                    manifest['eligible_counts'] = {key: len(rows) for key, rows in records.items()}
                    save_batch(manifest_path, manifest)
                if row['counted']:
                    print(f'Counted: {row["stop_reason"]}; {name}={len(records[name])}/{baseline["target_per_map"]}', flush=True)
                    break
                if attempt == args.max_startup_attempts:
                    raise RuntimeError('Repeated zero-map startup failures; fix map/TF initialization before continuing')
                print('Startup failed; retained artifacts. Waiting 30 seconds before retry...', flush=True)
                time.sleep(30)
            if index < len(plan):
                print('Waiting 30 seconds before the next run...', flush=True)
                time.sleep(30)
        manifest['status'] = 'completed'
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
        print('Interrupted; current experiment cleaned up. Rerun to supplement only missing samples.', flush=True)
    except Exception as exc:
        manifest['status'] = 'error'
        manifest['error'] = str(exc)
        print(f'Batch stopped: {exc}', flush=True)
    finally:
        save_batch(manifest_path, manifest)
        signal.signal(signal.SIGTERM, previous_handler)
    print('Batch summary:', manifest_path, flush=True)
    if manifest['status'] != 'completed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
