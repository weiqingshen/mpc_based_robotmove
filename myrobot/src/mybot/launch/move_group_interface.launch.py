from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_arm_description", package_name="mybot").to_moveit_configs()

    # 设置 use_sim_time 参数为 True
    sim_time_param = {"use_sim_time": True}

    # MoveGroupInterface demo 可执行文件
    move_group_demo = Node(
         name="move_group_interface",
         package="mybot",
         executable="move_group_interface",
         output="screen",
         parameters=[
             moveit_config.robot_description,
             moveit_config.robot_description_semantic,
             moveit_config.robot_description_kinematics,
             sim_time_param,  # 添加 use_sim_time 参数
         ],
     )


    return LaunchDescription([move_group_demo])

