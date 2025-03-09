#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "Trajectory.hpp"
#include <string>
#include "std_msgs/msg/float32.hpp"

class TargetPose {
public:
    // 通过提供目标姿态名称从 YAML 文件中加载姿态
    TargetPose(const rclcpp::Node::SharedPtr& node,
               moveit::planning_interface::MoveGroupInterface& move_group,
               const std::string& pose_name);

    // 通过直接提供位置和四元数来设置目标姿态
    TargetPose(const rclcpp::Node::SharedPtr& node,
               moveit::planning_interface::MoveGroupInterface& move_group,
               double x, double y, double z, double qx, double qy, double qz, double qw);

    geometry_msgs::msg::Pose getTargetPose() const;

    // 直接设定目标姿态
    void setTargetPose(double x, double y, double z, double qx, double qy, double qz, double qw);

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface& move_group_;
    geometry_msgs::msg::Pose target_pose_;
    std::string pose_name_;

    // 加载 YAML 文件中的参数
    bool loadPoseFromConfig(const YAML::Node& config);
    bool isloadTargetPose();
    void setTargetPose();

    // 默认 YAML 文件路径
    const std::string default_yaml_path_ = "../config/param.yaml";
};


class Object {
public:
    // 构造函数，读取 YAML 文件中的形状定义
    Object(const rclcpp::Node::SharedPtr& node, const std::string& object_name,
           const std::string& yaml_file_path = "../config/param.yaml");

    // 设置物体的位置
    void setPosition(float x,float y,float z);

    // 获取物体的碰撞对象
    moveit_msgs::msg::CollisionObject getCollisionObject() const;

    // 将物体添加到规划场景中
    void addToPlanningScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

private:
    rclcpp::Node::SharedPtr node_;
    shape_msgs::msg::SolidPrimitive primitive_;
    geometry_msgs::msg::Pose pose_;
    std::string object_name_;
    std::string yaml_file_path_;

    // 从 YAML 配置中加载形状
    bool loadShapeFromConfig(const YAML::Node& config);
};

// GraspPose 类
class GraspPose {
public:
    // 构造函数
    GraspPose(const rclcpp::Node::SharedPtr& node, const std::string& grasp_name,
              const std::string& yaml_file_path = "../config/param.yaml");

    // 获取抓取偏移姿态
    geometry_msgs::msg::Pose getGraspOffsetPose(const geometry_msgs::msg::Pose& object_pose) const;
    geometry_msgs::msg::Pose getObjectOffsetPose(const geometry_msgs::msg::Pose& object_pose) const;

private:
    rclcpp::Node::SharedPtr node_;
    geometry_msgs::msg::Pose offset_pose_;  // 偏移和方向
    std::string grasp_name_;
    std::string yaml_file_path_;

    // 从 YAML 配置中加载抓取偏移
    bool loadGraspOffsetFromConfig(const YAML::Node& config);
};


// ManipulatorAction 类
class ManipulatorAction {
public:
    ManipulatorAction(const rclcpp::Node::SharedPtr& node,
                      moveit::planning_interface::MoveGroupInterface& move_group,
                      moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                      moveit::planning_interface::MoveGroupInterface& gripper_group);

    // 执行动作
    void executeAction(Object& object, TargetPose& target_pose, GraspPose& grasp_pose, const std::string& trajectory_name = "") ;

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface& move_group_;
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface& gripper_group_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_state_publisher_;  // 发布器
    Trajectory trajectory_;  // 成员变量，定义一个 TrajectoryInterpolator 实例

    void closeGripper();
    void openGripper();
    void attachObject(Object& object);
    void detachObject(Object& object);
    void moveToPose(const geometry_msgs::msg::Pose& pose);


};
#endif // ARM_CONTROL_H
