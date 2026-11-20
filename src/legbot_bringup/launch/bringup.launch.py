"""Map-localized navigation with GO2 simulation or external platform topics."""
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration as Arg

from legbot_platform.navigation import include
from legbot_platform.map_profiles import SIMULATION_MAP, load_map, maps_directory, select_map
from legbot_platform.resources import robot_path


def setup(context):
    name = Arg('robot').perform(context)
    profile = Arg('robot_profile').perform(context) or robot_path(name)
    backend = Arg('backend').perform(context)
    map_name = Arg('map').perform(context) or (SIMULATION_MAP if backend == 'sim' else 'building2_9')
    building = select_map(map_name, Arg('map_profile').perform(context), Arg('maps_dir').perform(context))
    web = Arg('web_console').perform(context) == 'true'
    site = load_map(building, web=web, demo=backend == 'sim' and not web)
    common = {'robot_profile': profile, 'map_profile': str(building), 'web_console': Arg('web_console'), 'web_port': Arg('web_port')}
    for key in ('finish_dist', 'goal_settle_time'):
        value = context.launch_configurations.get(key) or site.get('navigation', {}).get(key)
        if value is not None:
            value = float(value)
            if not math.isfinite(value) or value < 0 or (key == 'finish_dist' and value == 0):
                raise ValueError(f'{building}: invalid {key}')
            common[key] = str(value)
    actions = [LogInfo(msg=f"Robot profile: {profile} | backend: {backend} | map: {site.get('id', building.parent.name)} @ {site.get('version', 'unversioned')} ({building})")]
    if backend == 'external':
        actions.append(include('navigation.launch.py', {**common, 'use_sim_time': Arg('use_sim_time')}))
    else:
        if name != 'go2':
            raise ValueError(f'No simulation backend registered for {name}')
        simulation = site.get('simulation', {})
        options = {}
        if 'world' in simulation:
            if not simulation.get('world_name'):
                raise ValueError(f'{building}: simulation.world requires the SDF world_name')
            options['world'] = str((building.parent / simulation['world']).resolve(strict=True))
            options['world_name'] = simulation['world_name']
        for axis, value in simulation.get('spawn', {}).items():
            if axis not in ('x', 'y', 'z', 'yaw') or not math.isfinite(float(value)):
                raise ValueError(f'{building}: invalid simulation.spawn.{axis}')
            options['spawn_' + axis] = str(value)
        actions.append(include('pct_cross_floor_demo.launch.py', {
            **common, 'use_sim_time': 'true', 'navigation_source': 'map_localized', 'publish_ground_truth': 'false',
            'gui': Arg('gui'), 'rviz': Arg('rviz'), **options,
        }))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='go2'),
        DeclareLaunchArgument('backend', default_value='sim', choices=['sim', 'external']),
        DeclareLaunchArgument('robot_profile', default_value=''),
        DeclareLaunchArgument('map', default_value='', description=f'Map directory name (sim: {SIMULATION_MAP}, external: building2_9)'),
        DeclareLaunchArgument('maps_dir', default_value=str(maps_directory())),
        DeclareLaunchArgument('map_profile', default_value='', description='Explicit map.yaml path, takes precedence over map'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Clock for external input', choices=['true', 'false']),
        DeclareLaunchArgument('web_console', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('web_port', default_value='8765'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
