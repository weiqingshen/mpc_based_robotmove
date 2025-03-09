#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "Trajectory.hpp"

class TrajectoryInterpolator {
public:
    // 构造函数只接受 YAML 文件路径，轨迹名称将在调用时传递
    explicit TrajectoryInterpolator(const std::string& yaml_file = "default_trajectory.yaml")
            : trajectory_(yaml_file) {}  // 加载 YAML 文件，但不指定具体的轨迹

    // 获取指定轨迹的关节数
    int getJointCount(const std::string& trajectory_name) const {
        std::vector<TrajectoryPoint> trajectory_points = trajectory_.getTrajectory(trajectory_name);
        if (!trajectory_points.empty()) {
            // 获取第一个点的关节数
            return trajectory_points[0].joint_positions.size();
        }
        // 如果没有轨迹点，返回 0
        return 0;
    }

    void executeTrajectory(moveit::planning_interface::MoveGroupInterface& move_group,
                           const std::string& trajectory_name,
                           size_t num_interpolation_points) {
        // 从 YAML 文件中获取指定的轨迹
        trajectory_points_ = trajectory_.getTrajectory(trajectory_name);

        // 获取 MoveGroup 的关节数和轨迹的关节数
        auto joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());
        size_t num_joints_in_move_group = joint_model_group->getVariableCount();
        size_t num_joints_in_trajectory = trajectory_points_[0].joint_positions.size();

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

        // 创建 JointTrajectory 消息并将其赋给 RobotTrajectory
        trajectory_msgs::msg::JointTrajectory& joint_trajectory = msg_trajectory.joint_trajectory;

        // 获取 MoveGroup 的所有关节名字
        const auto& joint_names = joint_model_group->getVariableNames();

        // 遍历每对相邻的轨迹点，执行插值
        for (size_t i = 0; i < trajectory_points_.size() - 1; ++i) {
            for (size_t j = 0; j <= num_interpolation_points; ++j) {
                double t = static_cast<double>(j) / (num_interpolation_points + 1);  // 均匀分布插值
                std::vector<double> interpolated_positions = getInterpolatedPoint(i, t);

                // 打印插值结果的大小
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Interpolated positions size: %zu", interpolated_positions.size());

                // 创建一个 JointTrajectoryPoint，填充位置数据
                trajectory_msgs::msg::JointTrajectoryPoint point;

                // 将 double 转换为 float 并赋值
                point.positions.resize(interpolated_positions.size());
                std::transform(interpolated_positions.begin(), interpolated_positions.end(),
                               point.positions.begin(), [](double val) { return static_cast<float>(val); });

                // 设置合理的速度和加速度
                point.velocities.resize(interpolated_positions.size(), 0.1f);  // 设置一个小的默认速度
                point.accelerations.resize(interpolated_positions.size(), 0.1f);  // 设置一个小的默认加速度

                // 设置时间戳
                point.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(j) / (num_interpolation_points + 1));

                joint_trajectory.points.push_back(point);
            }
        }

        // 处理最后一个轨迹点
        std::vector<double> final_positions = trajectory_points_.back().joint_positions;

        // 创建并添加最后一个轨迹点
        trajectory_msgs::msg::JointTrajectoryPoint final_point;
        final_point.positions.resize(final_positions.size());
        std::transform(final_positions.begin(), final_positions.end(),
                       final_point.positions.begin(), [](double val) { return static_cast<float>(val); });

        // 设置合理的速度和加速度
        final_point.velocities.resize(final_positions.size(), 0.1f);
        final_point.accelerations.resize(final_positions.size(), 0.1f);

        // 设置最后一个点的时间戳
        final_point.time_from_start = rclcpp::Duration::from_seconds(1.0);  // 为最后一个点设置一个合理的时间戳

        joint_trajectory.points.push_back(final_point);

        // 使用 MoveGroupInterface::Plan 执行轨迹
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = msg_trajectory;

        // 执行整个轨迹
        move_group.execute(plan);  // 一次性执行整个轨迹
    }





private:
    // 获取两个相邻轨迹点之间，给定比例 t 的插值结果
    // 参数 i 表示起始点的索引，t 表示从起始点到终点的比例（0 <= t <= 1）
    std::vector<double> getInterpolatedPoint(size_t start_idx, double t) const {
        if (start_idx >= trajectory_points_.size() - 1) {
            throw std::out_of_range("Invalid trajectory segment index");
        }

        // 获取起始点和终点
        const auto& start_point = trajectory_points_[start_idx];
        const auto& end_point = trajectory_points_[start_idx + 1];

        // 使用线性插值来计算中间位置
        std::vector<double> interpolated_positions;
        for (size_t i = 0; i < start_point.joint_positions.size(); ++i) {
            double interpolated_value = start_point.joint_positions[i] +
                                        t * (end_point.joint_positions[i] - start_point.joint_positions[i]);
            interpolated_positions.push_back(interpolated_value);
        }
        return interpolated_positions;
    }

    Trajectory trajectory_;  // YAML 文件加载器
    std::vector<TrajectoryPoint> trajectory_points_;  // 存储选定轨迹的点
};
