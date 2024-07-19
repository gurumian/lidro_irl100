# #!/usr/bin/env python3

import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidro_filter')
    pkg_driver_share = get_package_share_directory('lidro_driver')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    lidro_driver = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource([os.path.join(
                    pkg_driver_share, 'launch','lidro_driver.launch.py'
                )]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items()
    )

    filter = launch_ros.actions.Node(
        package="lidro_filter",
        executable="lidro_filter_node",
        parameters=[
            os.path.join(pkg_share, 'config', 'lidro_filter.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('point2_in', 'out'),
            ('point2_filtered', 'out_filtered')
        ],
        output="both",
    )

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    ])

    ld.add_action(lidro_driver)
    ld.add_action(filter)

    return ld
