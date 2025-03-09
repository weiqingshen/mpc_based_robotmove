cmake_minimum_required(VERSION 3.22)
project(mybot)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)


# 手动设置 serial 库的路径
#include_directories(/home/fins/ws_moveit2/src/serial/include)
#link_directories(/home/fins/ws_moveit2/src/serial/build/devel/lib)

# 添加可执行文件
add_executable(move_group_interface src/move_group_interface.cpp)
add_executable(end_effector_feedback src/end_effector_feedback.cpp)
add_executable(set_target_pose src/set_target_pose.cpp)
add_executable(execute src/execute.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_storage src/arm_execute_storage.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_processing src/arm_execute_processing.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_circle src/arm_execute_circle.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_circle_trajectory src/arm_execute_circle_trajectory.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_processing_trajectory src/arm_execute_processing_trajectory.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_storage_trajectory src/arm_execute_storage_trajectory.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_second_storage_trajectory src/arm_execute_second_storage_trajectory.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_total_trajectory src/arm_execute_total_trajectory.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_total_trajectory_final src/arm_execute_total_trajectory_final.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_total_trajectory_second src/arm_execute_total_trajectory_second.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_total_trajectory_second_final src/arm_execute_total_trajectory_second_final.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_node src/arm_execute.cpp src/arm_control.cpp
        src/Communication.h
        src/Communication.cpp
        src/Trajectory.cpp)
add_executable(trajectory_buffer_node src/trajectory_buffer.cpp)

# 链接必要的 ROS 2 和 MoveIt! 库（移除 yaml-cpp）
ament_target_dependencies(trajectory_buffer_node
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        tf2
        tf2_eigen
        tf2_ros
        yaml-cpp
)

# 链接依赖库
ament_target_dependencies(set_target_pose
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(move_group_interface
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(end_effector_feedback
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(arm_execute_node
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(execute
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(arm_execute_storage
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(arm_execute_processing
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_circle
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(arm_execute_circle_trajectory
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_total_trajectory
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_total_trajectory_final
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_total_trajectory_second
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_total_trajectory_second_final
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_processing_trajectory
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_storage_trajectory
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_second_storage_trajectory
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
# 安装可执行文件
install(TARGETS move_group_interface end_effector_feedback execute set_target_pose arm_execute_node trajectory_buffer_node arm_execute_storage arm_execute_processing arm_execute_storage_trajectory arm_execute_second_storage_trajectory arm_execute_processing_trajectory arm_execute_total_trajectory_final arm_execute_total_trajectory_second arm_execute_total_trajectory_second_final arm_execute_total_trajectory arm_execute_circle_trajectory arm_execute_circle
        DESTINATION lib/${PROJECT_NAME})

# 安装src目录中的所有文件
install(DIRECTORY src/ DESTINATION share/${PROJECT_NAME}/src
        FILES_MATCHING PATTERN "*.cpp" PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.c"
)

# 安装config目录中的所有文件
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 安装launch目录和其他文件
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
        PATTERN "setup_assistant.launch" EXCLUDE)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()