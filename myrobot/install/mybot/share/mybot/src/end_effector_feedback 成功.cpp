#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("end_effector_feedback");

void feedbackHandEndEffectorPosition(moveit::planning_interface::MoveGroupInterface& move_group) {
    rclcpp::WallRate loop_rate(0.2);  // 设置反馈频率为0.2Hz，每秒反馈一次
    while (rclcpp::ok()) {
        // 获取当前末端执行器的姿态
        auto current_pose = move_group.getCurrentPose("link6");

        // 反馈末端执行器的位置
        RCLCPP_INFO(LOGGER, "End Effector Position (hand): [x: %f, y: %f, z: %f]",
                    current_pose.pose.position.x, 
                    current_pose.pose.position.y, 
                    current_pose.pose.position.z);

        loop_rate.sleep();  // 控制循环频率为1Hz
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // 初始化ROS2节点系统

    // 创建节点，并启用 use_sim_time 参数
    auto node = rclcpp::Node::make_shared("end_effector_feedback_node", 
        rclcpp::NodeOptions().parameter_overrides({
            {"use_sim_time", true}  // 启用仿真时间
        })
    );

    // 创建MoveGroupInterface对象，用于控制arm规划组
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "arm");

    // 启动状态监视器
    move_group_interface.startStateMonitor();

    // 等待状态监视器完成同步
    rclcpp::sleep_for(std::chrono::seconds(2));  // 等待2秒，确保状态同步完成

    // 创建单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // 启动一个线程执行spin
    std::thread([&executor]() { executor.spin(); }).detach();

    // 调用反馈函数，每秒反馈末端执行器的位置
    feedbackHandEndEffectorPosition(move_group_interface);

    rclcpp::shutdown();  // 关闭ROS2节点系统
    return 0;
}
