"""Sample a running Linux navigation launch tree as JSON lines, until the launch exits."""

import argparse
import json
import os
import time
from pathlib import Path


def processes():
    result = {}
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            fields = (proc / 'stat').read_text().rsplit(') ', 1)[1].split()
            if fields[0] != 'Z':
                result[int(proc.name)] = (int(fields[1]), fields[19], int(fields[11]) + int(fields[12]))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return result


def sample(tree, pid, previous, elapsed, ticks):
    pending = [pid]
    rows = []
    while pending:
        current = pending.pop()
        pending.extend(child for child, info in tree.items() if info[0] == current)
        proc = Path('/proc') / str(current)
        _, start, cpu = tree[current]
        try:
            name = (proc / 'comm').read_text().strip()
            memory = (proc / 'smaps_rollup').read_text().splitlines()
            pss = next(int(line.split()[1]) for line in memory if line.startswith('Pss:'))
            rss = next(int(line.split()[1]) for line in memory if line.startswith('Rss:'))
        except (OSError, StopIteration):
            continue
        key = (current, start)
        cores = round((cpu - previous[key]) / ticks / elapsed, 3) if key in previous else None
        previous[key] = cpu
        rows.append(dict(pid=current, name=name, rss_kib=rss, pss_kib=pss, cpu_cores=cores))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pid', type=int, help='PID of the ros2 launch process')
    parser.add_argument('--interval', type=float, default=10.)
    args = parser.parse_args()
    if not 1 <= args.interval <= 600:
        parser.error('interval must be between 1 and 600 seconds')
    tree = processes()
    if args.pid not in tree:
        parser.error('launch process not found')
    identity = tree[args.pid][1]
    previous = {}
    started = last = time.monotonic()
    ticks = os.sysconf('SC_CLK_TCK')
    while tree.get(args.pid, (None, None))[1] == identity:
        now = time.monotonic()
        rows = sample(tree, args.pid, previous, now - last, ticks)
        print(json.dumps(dict(seconds=round(now-started, 2), pss_mib=round(sum(row['pss_kib'] for row in rows)/1024, 1),
                              processes=rows)), flush=True)
        last = now
        time.sleep(args.interval)
        tree = processes()


if __name__ == '__main__':
    main()
