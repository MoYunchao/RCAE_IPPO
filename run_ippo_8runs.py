#!/usr/bin/env python3
"""Run IPPO on default and maze40 four times each, with 30 seconds between runs."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parent
MODEL = 'models/inference/ippo/best_model.pt'
PLAN = [(map_name, repeat) for map_name in ('default', 'maze40') for repeat in range(1, 5)]


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


def main(*, plan=PLAN, battery_capacity='0.40', batch_folder='ippo_batches', description=__doc__):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--timeout-sec', type=float, default=1800,
                        help='Abort batch if a run has no mission summary within this many seconds')
    parser.add_argument('--dry-run', action='store_true', help='Print the plan without starting ROS')
    args = parser.parse_args()
    if not 0 < args.timeout_sec < float('inf'):
        parser.error('--timeout-sec must be finite and positive')
    for index, (map_name, repeat) in enumerate(plan, 1):
        print(f'[{index}/{len(plan)}] {map_name}, repeat {repeat}: ' + ' '.join(command_for(map_name, battery_capacity)), flush=True)
    print('Wait 30 seconds after process cleanup between runs.', flush=True)
    if args.dry_run:
        return
    if not (ROOT / 'install/setup.bash').is_file():
        parser.error('Build this workspace first: bash toBuild.sh')
    if not (ROOT / MODEL).is_file():
        parser.error(f'Model is missing: {MODEL}')

    def on_terminate(signum, frame):
        raise KeyboardInterrupt
    previous_handler = signal.signal(signal.SIGTERM, on_terminate)
    batch_dir = ROOT / 'output' / batch_folder / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    batch_dir.mkdir(parents=True)
    manifest = {'model': MODEL, 'repeats_per_map': 4, 'pause_seconds': 30,
                'battery_capacity_ah': [float(battery_capacity)] * 3, 'timeout_sec': args.timeout_sec,
                'status': 'running', 'runs': []}
    manifest_path = batch_dir / 'batch_summary.json'
    save_batch(manifest_path, manifest)
    print('Batch results:', batch_dir, flush=True)
    try:
        for index, (map_name, repeat) in enumerate(plan, 1):
            command = command_for(map_name, battery_capacity)
            directory = batch_dir / map_name / f'run_{repeat:02d}'
            row = {'map': map_name, 'repeat': repeat, 'command': command,
                   'directory': str(directory), 'status': 'running'}
            manifest['runs'].append(row)
            save_batch(manifest_path, manifest)
            print(f'[{index}/{len(plan)}] Starting {map_name} #{repeat}; log: {directory / "console.log"}', flush=True)
            try:
                row.update(run_once(command, directory, args.timeout_sec))
                row['status'] = 'completed'
            except BaseException as exc:
                row.update(status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'error',
                           error=str(exc) or type(exc).__name__)
                raise
            finally:
                save_batch(manifest_path, manifest)
            print(f'[{index}/{len(plan)}] Finished: {row["stop_reason"]}; processes stopped.', flush=True)
            if index < len(plan):
                print('Waiting 30 seconds before the next run...', flush=True)
                time.sleep(30)
        manifest['status'] = 'completed'
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
        print('Interrupted; this experiment has been stopped. No next run will start.', flush=True)
    except Exception as exc:
        manifest['status'] = 'error'
        print(f'Batch stopped: {exc}', flush=True)
    finally:
        save_batch(manifest_path, manifest)
        signal.signal(signal.SIGTERM, previous_handler)
    print('Batch summary:', manifest_path, flush=True)
    if manifest['status'] != 'completed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
