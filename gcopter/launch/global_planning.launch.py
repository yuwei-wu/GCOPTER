from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    gcopter_config_path = PathJoinSubstitution(
        [FindPackageShare('gcopter'), 'config', 'global_planning.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('gcopter'), 'config', 'global_planning.rviz']
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),

        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='pos_vel_plot',
            arguments=[
                '/visualizer/speed',
                '/visualizer/total_thrust',
                '/visualizer/tilt_angle',
                '/visualizer/body_rate'
            ],
            output='screen',
        ),

        Node(
            package='mockamap',
            executable='mockamap_node',
            name='mockamap_node',
            output='screen',
            parameters=[{
                'seed': 1024,
                'update_freq': 1.0,
                'resolution': 0.25,
                'x_length': 50,
                'y_length': 50,
                'z_length': 5,
                'type': 1,
                'complexity': 0.025,
                'fill': 0.3,
                'fractal': 1,
                'attenuation': 0.1,
            }],
            remappings=[('/mock_map', '/voxel_map')],
        ),

        Node(
            package='gcopter',
            executable='global_planning_node',
            name='global_planning_node',
            output='screen',
            parameters=[gcopter_config_path],
        ),
    ])
