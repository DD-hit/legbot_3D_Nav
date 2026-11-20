import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from legbot_platform.calibration import sensor_profile


def robot_path(name):
    if not re.fullmatch(r'[a-z][a-z0-9_]*', name):
        raise ValueError(f'Invalid robot name: {name!r}')
    path = Path(share('legbot_config'), 'robots', name + '.yaml')
    if not path.is_file():
        raise ValueError(f'Robot profile not found: {path}')
    return str(path)


def model_and_controller(profile_path, policy_override, backend):
    defaults = sensor_profile(robot_path('go2'))
    robot = sensor_profile(profile_path) if profile_path else defaults
    model = Path(share('robot_description'), robot.get('model', defaults['model']))
    directory = Path(share('legbot_config'), robot.get('control_directory', defaults['control_directory']))
    policy = policy_override or robot.get('policy_profile', defaults['policy_profile'])
    if not re.fullmatch(r'[A-Za-z0-9_]+', policy):
        raise ValueError(f'Invalid policy profile: {policy!r}')
    filename = 'gazebo.yaml' if backend == 'sim' else 'robot_control.yaml'
    controller = directory / (filename if policy == 'himloco' else policy + '_controller.yaml')
    for path in (model, controller):
        if not path.is_file():
            raise FileNotFoundError(path)
    return robot, str(model), str(controller)


def fastlio_path(profile_path, name):
    if name.startswith(('./', '../')):
        return str(Path(profile_path).absolute().parent / name)
    return str(Path(share('legbot_config'), 'sensors/fastlio', name))
