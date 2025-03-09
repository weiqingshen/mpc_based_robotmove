#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>  // MoveGroupInterface
#include <moveit/planning_scene_interface/planning_scene_interface.h>  // 可选，若涉及规划场景
#include <moveit_msgs/msg/display_trajectory.hpp>  // DisplayTrajectory 消息
#include <moveit_msgs/msg/robot_trajectory.hpp>    // RobotTrajectory 消息
#include <trajectory_msgs/msg/joint_trajectory.hpp>  // JointTrajectory 消息
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>  // JointTrajectoryPoint 消息

struct TrajectoryPoint {
    std::string name;
    std::vector<double> joint_positions;
    std::vector<double> velocities;      // 速度
    std::vector<double> accelerations;   // 加速度
    std::vector<double> time_from_start; // 从起始时间
};

class Trajectory {
public:
    // 构造函数，加载 YAML 文件
    Trajectory(const std::string& yaml_file);

    // 根据轨迹名获取轨迹
    const std::vector<TrajectoryPoint>& getTrajectory(const std::string& trajectory_name) const;

    // 执行轨迹
    bool executeTrajectory(moveit::planning_interface::MoveGroupInterface& move_group,
                           const std::string& trajectory_name);

    void setYamlFileName(const std::string& new_yaml_file);
private:
    // 存储所有的轨迹，按轨迹名称索引
    std::unordered_map<std::string, std::vector<TrajectoryPoint>> trajectories_;
    std::string getDefaultYamlPath(const std::string& yaml_file);
};

