"""Check installed map selection, legacy aliases and per-site launch settings."""
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from legbot_platform.map_profiles import load_map, select_map


def selected_options(root, name, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(maps_dir=str(root), map=name, **overrides)
    launch = Path(share('legbot_bringup'), 'launch/bringup.launch.py')
    description = PythonLaunchDescriptionSource(str(launch)).get_launch_description(context)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    _, included = description.entities[-1].execute(context)
    return dict(included.launch_arguments)


def main():
    installed = Path(share('legbot_config'), 'maps')
    base = select_map('building2_9', '', installed)
    assert base == Path(share('legbot_bringup'), 'config/map.yaml').resolve()
    assert base == Path(share('legbot_config'), 'navigation/map.yaml').resolve()
    bundled = load_map(base)
    assert len(bundled['floors']) == 5
    current_sim = load_map(selected_options(installed, '')['map_profile'])
    assert current_sim['version'] == 'edit-9747665e6f72ae'
    assert [name for _, name in current_sim['floors']] == ['一楼', '二楼', '三楼', '四楼']
    assert selected_options(installed, '', backend='external')['map_profile'] == str(base)
    with tempfile.TemporaryDirectory() as folder:
        root = Path(folder)
        site = root / '办公区'
        site.mkdir()
        for name in ('source.pcd', 'planning.pickle', 'display.json', 'world.sdf'):
            (site / name).symlink_to(base.parent / name)
        (root / 'original').symlink_to(base.parent)
        profile = yaml.safe_load(base.read_text())
        profile.update(id='office', version='test-v2', floors=[[0, '大厅']], navigation={'finish_dist': 0.1, 'goal_settle_time': 0})
        profile['simulation']['spawn']['yaw'] = 0.8
        manifest = site / 'map.yaml'
        manifest.write_text(yaml.safe_dump(profile))
        assert load_map(select_map('办公区', '', root))['floors'] == [[0, '大厅']]
        options = selected_options(root, '办公区')
        assert options['map_profile'] == str(manifest) and options['spawn_yaw'] == '0.8'
        assert options['finish_dist'] == '0.1' and options['goal_settle_time'] == '0.0'
        assert selected_options(root, '办公区', finish_dist='0.12')['finish_dist'] == '0.12'
        assert selected_options(root, '办公区', map_profile=str(base))['map_profile'] == str(base)
        for invalid in ('missing', '../办公区'):
            try:
                select_map(invalid, '', root)
            except ValueError:
                pass
            else:
                raise AssertionError(invalid)
        (site / 'planning.pickle').unlink()
        try:
            load_map(manifest)
        except FileNotFoundError as error:
            assert 'tomogram' in str(error)
        else:
            raise AssertionError('missing planning map accepted')
    print('PASS named maps, installed aliases, independent metadata and options, overrides, missing resources')


if __name__ == '__main__':
    main()
