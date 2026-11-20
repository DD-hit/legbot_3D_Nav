import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory as share


SIMULATION_MAP = 'building_edited'


def maps_directory():
    directory = os.environ.get('LEGBOT_MAPS_DIR')
    return Path(directory).expanduser() if directory else Path(share('legbot_config'), 'maps')


def select_map(name, profile, directory):
    if profile:
        path = Path(profile).expanduser()
    else:
        if not name or Path(name).name != name or name in ('.', '..'):
            raise ValueError(f'Expected a map directory name, got {name!r}')
        root = Path(directory).expanduser()
        path = root / name / 'map.yaml'
        if not path.is_file():
            available = sorted(item.parent.name for item in root.glob('*/map.yaml') if item.is_file())
            raise ValueError(f'Map {name!r} not found in {root}; available maps: {", ".join(available)}')
    return path.resolve(strict=True)


def load_map(path, *, web=True, demo=False):
    manifest = Path(path).resolve(strict=True)
    with manifest.open(encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict):
        raise ValueError(f'{manifest}: expected a map profile')
    keys = ['frame', 'pcd', 'tomogram']
    if web:
        keys.extend(('id', 'version', 'display'))
    if demo and 'demo_tomogram' in profile:
        keys.append('demo_tomogram')
    for key in keys:
        if not isinstance(profile.get(key), str) or not profile[key].strip():
            raise ValueError(f'{manifest}: expected a nonempty {key}')
    floors = profile.get('floors')
    if web and (not isinstance(floors, list) or not floors):
        raise ValueError(f'{manifest}: expected at least one floor')
    last_height = -math.inf
    for floor in floors or []:
        if not (isinstance(floor, list) and len(floor) == 2 and type(floor[0]) in (int, float)
                and math.isfinite(floor[0]) and floor[0] > last_height and isinstance(floor[1], str) and floor[1].strip()):
            raise ValueError(f'{manifest}: expected ascending [height, floor name] pairs')
        last_height = floor[0]
    for key in ('pcd', 'tomogram', 'demo_tomogram', 'display'):
        if key not in keys:
            continue
        value = profile[key]
        planning = key in ('tomogram', 'demo_tomogram')
        filename = value + '.pickle' if planning and not value.endswith('.pickle') else value
        candidates = [manifest.parent / filename]
        if not Path(filename).is_absolute() and not value.startswith(('./', '../')):
            if planning:
                package = Path(share('pct_planner'))
                candidates.extend([package / 'planner/scripts' / os.environ.get('LEGBOT_DATA_DIR', 'data') / 'tomogram' / filename,
                                   package / 'src/tomogram' / filename])
            else:
                candidates.append(Path(share('pct_planner' if key == 'pcd' else 'legbot_bringup')) / filename)
        source = next((item for item in candidates if item.is_file()), None)
        if source is None:
            raise FileNotFoundError(f'{manifest}: missing {key}: {filename}')
        profile[key] = str(source.with_suffix('') if planning else source)
    return profile
