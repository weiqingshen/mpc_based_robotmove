//
// Created by fins on 24-11-6.
//

#include "Trajectory.hpp"

std::string Trajectory::getDefaultYamlPath(const std::string& yaml_file) {
    try {
        // 假设你使用 ament_index_cpp 库来动态获取包的路径
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("mybot");

        // 如果传入的文件名没有 .yaml 后缀，则添加
        std::string formatted_file = yaml_file;
        if (formatted_file.substr(formatted_file.find_last_of(".") + 1) != "yaml") {
            formatted_file += ".yaml";
        }

        return package_share_dir + "/config/" + formatted_file; // 拼接路径
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Object"), "Failed to find package path: %s", e.what());
        throw; // 抛出异常，确保调用者可以处理
    }
}



Trajectory::Trajectory(const std::string& yaml_file) {
    std::string full_path;  // 在try块外声明，确保在catch中使用
    try {
        // 只传递文件名，获取完整路径
        full_path = getDefaultYamlPath(yaml_file);

        // 读取 YAML 文件
        YAML::Node config = YAML::LoadFile(full_path);

        // 解析 YAML 文件中的 trajectories 节点
        for (const auto& trajectory_node : config["trajectories"]) {
            std::string trajectory_name = trajectory_node["name"].as<std::string>();
            std::vector<TrajectoryPoint> points;

            // 解析每个 trajectory 的 points
            for (const auto& point_node : trajectory_node["points"]) {
                TrajectoryPoint point;
                point.name = point_node["name"].as<std::string>();

                // 读取 joint_positions、velocities、accelerations 和 time_from_start
                point.joint_positions = point_node["joint_positions"].as<std::vector<double>>();
                //point.velocities = point_node["velocities"].as<std::vector<double>>();
                //point.accelerations = point_node["accelerations"].as<std::vector<double>>();
                point.time_from_start = point_node["time_from_start"].as<std::vector<double>>();

                points.push_back(point);
            }

            // 将轨迹存储到地图中，以轨迹名称为键
            trajectories_[trajectory_name] = points;
        }
    } catch (const YAML::BadFile& e) {
        // 错误处理，输出详细的错误信息
        std::cerr << "Error loading YAML file: " << full_path << std::endl;
        throw;  // 抛出异常，确保调用者处理
    } catch (const YAML::ParserException& e) {
        // 如果解析 YAML 时出错，输出详细错误信息
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        // 捕获其他异常并输出错误信息
        std::cerr << "Unexpected error: " << e.what() << std::endl;
        throw;
    }
}

void Trajectory::setYamlFileName(const std::string& new_yaml_file) {
    // 清空当前轨迹
    trajectories_.clear();

    // 使用新文件名加载 YAML 文件
    try {
        std::string full_path = getDefaultYamlPath(new_yaml_file);

        YAML::Node config = YAML::LoadFile(full_path);

        for (const auto& trajectory_node : config["trajectories"]) {
            std::string trajectory_name = trajectory_node["name"].as<std::string>();
            std::vector<TrajectoryPoint> points;

            for (const auto& point_node : trajectory_node["points"]) {
                TrajectoryPoint point;
                point.name = point_node["name"].as<std::string>();
                point.joint_positions = point_node["joint_positions"].as<std::vector<double>>();
                point.time_from_start = point_node["time_from_start"].as<std::vector<double>>();
                points.push_back(point);
            }
            trajectories_[trajectory_name] = points;
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to load new YAML file: " << e.what() << std::endl;
        throw;
    }
}




const std::vector<TrajectoryPoint>& Trajectory::getTrajectory(const std::string& trajectory_name) const {
    auto it = trajectories_.find(trajectory_name);
    if (it != trajectories_.end()) {
        return it->second;
    } else {
        throw std::runtime_error("Trajectory not found: " + trajectory_name);
    }
}
// 这一段代码在RViz里面有比较好的演示效果 但是实际在display_planned_path里面只发出了首尾点的坐标
bool Trajectory::executeTrajectory(moveit::planning_interface::MoveGroupInterface& move_group,
                                   const std::string& trajectory_name) {
    // 从 YAML 文件中获取指定的轨迹
    std::vector<TrajectoryPoint> trajectory_points = getTrajectory(trajectory_name);

    // 获取 MoveGroup 的关节数和轨迹的关节数
    auto joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());
    size_t num_joints_in_move_group = joint_model_group->getVariableCount();
    size_t num_joints_in_trajectory = trajectory_points[0].joint_positions.size();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "MoveGroup has %zu joints, trajectory has %zu joints.",
                num_joints_in_move_group, num_joints_in_trajectory);

    if (num_joints_in_move_group != num_joints_in_trajectory) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Joint count mismatch: MoveGroup has %zu joints, but trajectory has %zu joints.",
                     num_joints_in_move_group, num_joints_in_trajectory);
        throw std::runtime_error("Joint count mismatch between MoveGroupInterface and trajectory.");
    }

    // 创建 moveit_msgs::msg::RobotTrajectory 消息
    moveit_msgs::msg::RobotTrajectory msg_trajectory;
    trajectory_msgs::msg::JointTrajectory& joint_trajectory = msg_trajectory.joint_trajectory;

    // 获取 MoveGroup 的所有关节名字
    const auto& joint_names = joint_model_group->getVariableNames();

    // 遍历每个轨迹点并赋值
    for (const auto& point : trajectory_points) {
        trajectory_msgs::msg::JointTrajectoryPoint joint_point;
        joint_point.positions = point.joint_positions;  // 直接赋值位置
        // joint_point.velocities = point.velocities;      // 直接赋值速度
        // joint_point.accelerations = point.accelerations; // 直接赋值加速度

        // 直接赋值时间，无需转换
        joint_point.time_from_start.sec = static_cast<int32_t>(point.time_from_start[0]);
        joint_point.time_from_start.nanosec = static_cast<uint32_t>(point.time_from_start[1]);

        joint_trajectory.points.push_back(joint_point);
    }

    // 使用 MoveGroupInterface::Plan 执行轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.plan(plan);
    plan.trajectory_.joint_trajectory.points= msg_trajectory.joint_trajectory.points;

    // 执行整个轨迹
    moveit::core::MoveItErrorCode execution_result = move_group.execute(plan);
    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Execution failed.");
        return false;  // 如果执行失败，返回false
    }

    return true;  // 轨迹执行成功，返回true
}

// void Trajectory::executeTrajectory(moveit::planning_interface::MoveGroupInterface& move_group,
//                                    const std::string& trajectory_name) {
//     // 从 YAML 文件中获取指定的轨迹
//     std::vector<TrajectoryPoint> trajectory_points = getTrajectory(trajectory_name);
//
//     // 检查 move_group 中的关节数是否与轨迹中的一致
//     auto joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());
//     size_t num_joints_in_move_group = joint_model_group->getVariableCount();
//     size_t num_joints_in_trajectory = trajectory_points[0].joint_positions.size();
//
//     if (num_joints_in_move_group != num_joints_in_trajectory) {
//         throw std::runtime_error("Joint count mismatch between MoveGroupInterface and trajectory.");
//     }
//
//     // 遍历轨迹点，逐点执行
//     for (size_t i = 0; i < trajectory_points.size(); ++i) {
//         const auto& point = trajectory_points[i];
//
//         // 设置目标关节位置
//         move_group.setJointValueTarget(point.joint_positions);
//
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//                     "Executing trajectory point %zu/%zu: %s",
//                     i + 1, trajectory_points.size(), point.name.c_str());
//
//         // 执行当前点的运动
//         moveit::core::MoveItErrorCode result = move_group.move();
//         if (result != moveit::core::MoveItErrorCode::SUCCESS) {
//             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
//                          "Failed to execute trajectory point %zu: %s",
//                          i + 1, point.name.c_str());
//             throw std::runtime_error("Trajectory point execution failed.");
//         }
//
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//                     "Successfully executed trajectory point %zu/%zu: %s",
//                     i + 1, trajectory_points.size(), point.name.c_str());
//     }
// }
