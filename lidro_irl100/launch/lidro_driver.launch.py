import os

import ament_index_python.packages
import launch
import launch_ros.actions
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('lidro_irl100')

    config_directory = os.path.join(
        pkg_share,
        'config')
    params = os.path.join(config_directory, 'lidro.yaml')
    lidro_driver_node = launch_ros.actions.Node(package='lidro_irl100',
                                                   executable='lidro_irl100_node',
                                                   output='both',
                                                   parameters=[params])
    xacro_file = os.path.join(pkg_share,'urdf','robot.urdf.xacro')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
        }]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
            description='Flag to enable joint_state_publisher_gui'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        lidro_driver_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=lidro_driver_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
        )),
    ])
