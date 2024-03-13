import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    prefix = LaunchConfiguration("prefix")
    
    rviz_launch=    Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            namespace=prefix,
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        )
    
    uwb_subscriber_node = Node(
        package='uwb_subscriber',
        executable='uwb_subscriber_node',
        name='uwb_subscriber',
        namespace=prefix,
        respawn=True,
        respawn_delay=1.0,
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'}, 
            {'baud_rate': 115200}
            ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
            ]
    )

    return LaunchDescription(
        [
        uwb_subscriber_node,
        rviz_launch
        ]
    )