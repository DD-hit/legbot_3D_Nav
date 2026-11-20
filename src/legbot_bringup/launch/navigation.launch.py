"""Map-localized navigation with external odometry, sensors and platform control."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction

from legbot_platform.navigation import navigation_actions, navigation_arguments, read_profile


def setup(context):
    return navigation_actions(context, read_profile(context, 'robot_profile'))


def generate_launch_description():
    return LaunchDescription([*navigation_arguments('true', 'false'), OpaqueFunction(function=setup)])
