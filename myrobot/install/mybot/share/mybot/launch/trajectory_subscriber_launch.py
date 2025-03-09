from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mybot',  # 请确保这里的包名正确
            executable='trajectory_subscriber_node',
            name='trajectory_subscriber',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},  # 你的串口设置
                {'baud_rate': 921600}
            ]
        ),
    ])

