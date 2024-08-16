from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
   return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/mnt/c/temp/lexus3sample02.mcap', '--clock', '--loop'],
            output='screen'
        ),
        Node(
            package='point_cloud_filter',
            executable='point_cloud_filter',
            name='point_cloud_filter',
            output='screen',
        ),
        
   ])
