"""Run against a fresh, headless building2_9 simulation with web_console enabled."""

import argparse
import json
import time
import uuid
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--soak-seconds', type=float, default=0.0,
                        help='repeat localized entrance-to-upper round trips for this wall-clock duration')
    parser.add_argument('--url', default='http://127.0.0.1:8765')
    parser.add_argument('--upper', action='store_true', help='use the longer upper-platform round trip')
    parser.add_argument('--entry-repair', action='store_true', help='walk to two verified former entry-layer holes')
    parser.add_argument('--round-trip-only', action='store_true', help='navigate from the entry to the upper platform and return')
    args = parser.parse_args()

    def call(path, payload=None):
        data = None if payload is None else json.dumps(payload).encode()
        req = Request(args.url + '/api/' + path, data=data, headers={'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=10) as stream:
                return json.load(stream)
        except HTTPError as error:
            raise RuntimeError(error.read().decode()) from error

    def wait(predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            state = call('state')
            if predicate(state):
                return state
            time.sleep(0.5)
        raise AssertionError(f'timed out: {state["task"]}, {state["localization"]}')

    def submit(points):
        task_id = uuid.uuid4().hex
        call('tasks', {'id': task_id, 'points': [dict(yaw=0., **point) for point in points]})
        return task_id

    def terminal(task_id, expected, completed, seconds=550):
        final = wait(lambda state: state['task']['id'] == task_id and state['task']['phase'] in
                     ('succeeded', 'failed', 'canceled'), seconds)['task']
        assert (final['phase'], final['index']) == (expected, completed), final
        print(f'PASS {expected}, {completed}/{len(final["points"])} stops, {final["reason"]}', flush=True)

    entry = dict(x=-5.5, y=6.0, floor=0)
    nearby = dict(x=-5.5, y=5.5, floor=0)
    upper = dict(x=2., y=-3., floor=3)
    island = dict(x=-7.0453, y=-0.9868, floor=4)
    state = wait(lambda state: state.get('standing'), 100)
    assert state['task']['phase'] == 'idle' and not state['ready'], 'use a fresh simulation at the entry'
    call('seed', dict(**entry, yaw=0.05))
    state = wait(lambda state: state['ready'], 40)
    assert state['task']['phase'] == 'idle' and not state['route']
    if args.round_trip_only or args.soak_seconds > 0:
        deadline = time.monotonic() + args.soak_seconds
        while True:
            terminal(submit([upper, entry]), 'succeeded', 2, 1500)
            if time.monotonic() >= deadline:
                break
        return
    terminal(submit([nearby]), 'succeeded', 1)
    if args.entry_repair:
        repaired = [dict(x=-4.6453, y=6.6132, floor=0), dict(x=-5.4453, y=7.2132, floor=0)]
        terminal(submit(repaired + [entry]), 'succeeded', 3)
    platform = upper if args.upper else dict(x=0.25, y=-4.50, floor=1)
    terminal(submit([entry, island]), 'failed', 1)
    task_id = submit([upper, entry])
    wait(lambda state: state['task']['id'] == task_id and state['task']['phase'] == 'walking', 60)
    time.sleep(2)
    call('cancel', dict(id=task_id))
    terminal(task_id, 'canceled', 0, 20)
    terminal(submit([entry]), 'succeeded', 1)
    terminal(submit([platform, entry]), 'succeeded', 2)


if __name__ == '__main__':
    main()
