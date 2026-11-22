"""Persistent D1 speed ceilings, snapshotted per manual gesture or navigation task."""
import json
import math
import os
import tempfile
from pathlib import Path

FILE = Path('/workspace/d1_localization/speed-settings.json')
MODES = ('manual', 'navigation')
BOUNDS = {'forward': (.05, 3.0), 'lateral': (.10, 1.0), 'yaw': (.02, 3.0)}
DEFAULT = {'forward': .12, 'lateral': .12, 'yaw': .25}

def validate(settings):
    if not isinstance(settings, dict) or set(settings) != set(MODES):
        raise ValueError('速度配置缺少控制模式')
    for values in settings.values():
        if not isinstance(values, dict) or set(values) != set(BOUNDS):
            raise ValueError('速度参数不完整')
        for name, (low, high) in BOUNDS.items():
            value = values[name]
            if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError('速度超出允许范围')
    return settings

def load():
    try:
        settings = json.loads(FILE.read_text())
        for mode in MODES:
            if isinstance(settings, dict) and isinstance(settings.get(mode), dict):
                settings[mode].setdefault('lateral', .12)
        return validate(settings)
    except FileNotFoundError:
        return {mode: DEFAULT.copy() for mode in MODES}

def save(settings):
    validate(settings)
    with tempfile.NamedTemporaryFile(mode='w', dir=FILE.parent, delete=False) as f:
        try:
            json.dump(settings, f)
            f.flush()
            os.fsync(f.fileno())
            os.replace(f.name, FILE)
        finally:
            Path(f.name).unlink(missing_ok=True)

if __name__ == '__main__':
    limits = load()['navigation']
    import sys
    if sys.argv[1:] == ['--axes']:
        print(limits['forward'], limits['lateral'], limits['yaw'])
    else:
        print(limits['forward'], limits['yaw'])
