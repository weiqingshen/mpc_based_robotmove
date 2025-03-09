#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("set_target_pose");

void setTargetPose(moveit::planning_interface::MoveGroupInterface& move_group) {
    // 创建一个目标位姿对象
    geometry_msgs::msg::Pose target_pose;
    //target_pose.orientation.w = 1.0;  // 假设目标的姿态为单位四元数
    target_pose.position.x =  -0.001686;  // 目标位置的X坐标
    target_pose.position.y =  0.013655;  // 目标位置的Y坐标
    target_pose.position.z = 0.620242;  // 目标位置的Z坐标

    // 设置目标位姿
    move_group.setPoseTarget(target_pose, "link6");

    // 规划和执行
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(LOGGER, "规划成功，开始执行运动。");
        move_group.move();
    } else {
        RCLCPP_WARN(LOGGER, "规划失败。");
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // 初始化ROS2节点系统

    // 创建节点，并启用 use_sim_time 参数
    auto node = rclcpp::Node::make_shared("set_target_pose_node", 
        rclcpp::NodeOptions().parameter_overrides({
            {"use_sim_time", true}  // 启用仿真时间
        })
    );

    // 创建MoveGroupInterface对象，用于控制arm规划组
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

    // 启动状态监视器
    move_group.startStateMonitor();

    // 等待状态监视器完成同步
    rclcpp::sleep_for(std::chrono::seconds(2));  // 等待2秒，确保状态同步完成

    // 设置并执行目标位姿
    setTargetPose(move_group);

    rclcpp::shutdown();  // 关闭ROS2节点系统
    return 0;
}
