"""Exercise manual seeds and oriented waypoints in a fresh Web simulation at the entry."""

import json
import math
import time
import uuid
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def call(path, payload=None):
    data = json.dumps(payload).encode() if payload is not None else None
    request = Request('http://127.0.0.1:8765/api/' + path, data=data,
                      headers={'Content-Type': 'application/json'})
    with urlopen(request, timeout=5) as stream:
        return json.load(stream)


def wait(predicate, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        state = call('state')
        if predicate(state):
            return state
        time.sleep(0.15)
    raise AssertionError(state)


def locate(point, succeeds):
    call('seed', point)
    pending = wait(lambda state: state['localization']['state'] in ('COLLECTING', 'MATCHING'), 4)
    assert pending['pose'] is None and not pending['ready']
    assert call('seed', point)['accepted']
    state = wait(lambda state: state['ready'] or state['localization']['state'] == 'FAILED', 38)
    print('seed', point, state['localization'], 'ready', state['ready'], flush=True)
    assert state['ready'] == succeeds
    if not succeeds:
        assert state['pose'] is None
        try:
            call('tasks', dict(id=uuid.uuid4().hex, points=[point]))
        except HTTPError as error:
            assert error.code == 409
        else:
            raise AssertionError('navigation allowed after failed localization')
    return state


def navigate(points):
    task_id = uuid.uuid4().hex
    call('tasks', dict(id=task_id, points=points))
    for index, point in enumerate(points):
        result = wait(lambda state: state['task']['id'] == task_id and
                      (state['task']['index'] > index or state['task']['phase'] in ('failed', 'canceled')), 180)
        task = result['task']
        assert task['index'] == index + 1, task
        pose = result['pose']
        error = abs(math.degrees(math.remainder(pose[3] - point['yaw'], 2 * math.pi)))
        distance = math.hypot(pose[0] - point['x'], pose[1] - point['y'])
        print('arrival', point, pose, 'xy error', distance, 'heading error deg', error, flush=True)
        assert distance <= .08 and error <= 6
    assert task['phase'] == 'succeeded'


if __name__ == '__main__':
    entry = dict(x=-5.5, y=6., floor=0, yaw=0.)
    assert wait(lambda state: state.get('standing'), 100)['task']['phase'] == 'idle'
    locate(entry, True)
    locate(dict(entry, x=2., y=-3.), False)
    locate(dict(entry, yaw=math.pi), False)
    locate(dict(entry, floor=3), False)
    fixed = locate(dict(entry, x=-5.15, yaw=math.radians(15)), True)['pose']
    navigate([dict(entry, x=fixed[0], y=fixed[1], yaw=math.pi/2)])
    navigate([dict(entry, x=-5.5, y=6.6, yaw=0.), dict(entry, yaw=-math.pi/2)])
