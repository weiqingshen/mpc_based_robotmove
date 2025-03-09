#include "arm_control.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "Communication.h"
#include <queue>

using namespace std;

int main(int argc, char** argv) {
    rclcpp::init(argc,argv);// 初始化ROS2节点系统，使系统可以处理ROS2的参数和通信功能
    rclcpp::NodeOptions node_options;//
    node_options.automatically_declare_parameters_from_overrides(true);

    // 创建节点 命名为move_group_interface
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto communication_node = std::make_shared<Communication>();
    // 创建一个单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;// 添加节点到执行器中，以便使其可以处理回调
    executor.add_node(move_group_node);// 在一个新线程中启动执行器的事件循环，以便节点可以异步处理事件。
    executor.add_node(communication_node);

    std::thread([&]() { executor.spin(); }).detach();

    //实例化
    static const std::string PLANNING_GROUP = "arm";
    static const std::string GRIPPER_GROUP = "hand";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    move_group.setMaxVelocityScalingFactor(1);  // 增加速度到 1 倍

    // 创建 Object, TargetPose, GraspPose 实例
    Object object1(move_group_node, "cylinder1");
    Object object2(move_group_node, "cylinder2");
    Object object3(move_group_node, "cylinder3");
    TargetPose place_pose1(move_group_node, move_group, "place_pose1");
    TargetPose place_pose2(move_group_node, move_group, "place_pose2");
    TargetPose place_pose3(move_group_node, move_group, "place_pose3");
    GraspPose grasp_pose(move_group_node, "grasp1");

    position target1;
    position target2;
    queue<int> order_queue;

    // 获取机器人模型组
    auto joint_model_group = move_group.getCurrentState()->getJointModelGroup(move_group.getName());
    size_t num_joints_in_move_group = joint_model_group->getVariableCount();

// 打印关节数和关节名称
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MoveGroup has %zu joints", num_joints_in_move_group);
    const auto& joint_names = joint_model_group->getVariableNames();
    for (const auto& joint_name : joint_names) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint: %s", joint_name.c_str());
    }
    // 调用类中的阻塞方法等待消息
    // node->waitForMessage();

    // 获取接收到的目标位置信息并赋值给 target1
    // node->getPosition(target1);

    object1.setPositionFromYaml("pose1");
    object2.setPositionFromYaml("pose2");
    object3.setPositionFromYaml("pose3");
    // 获取 CollisionObject 并添加到场景中
    collision_objects.push_back(object1.getCollisionObject());
    collision_objects.push_back(object2.getCollisionObject());
    collision_objects.push_back(object3.getCollisionObject());
    // 批量添加物体到场景中
    planning_scene_interface.addCollisionObjects(collision_objects);

    ManipulatorAction manipulator_action(move_group_node, move_group, planning_scene_interface, gripper_group);
    manipulator_action.moveAction(object1,grasp_pose,"processing11");

    communication_node->waitForMessage();

    communication_node->getPosition(target1);

    place_pose1.setTargetPose(target1.x1, target1.y1, 0.224, 0, 1, 0, 0);
    place_pose2.setTargetPose(target1.x2, target1.y2, 0.224, 0, 1, 0, 0);
    place_pose3.setTargetPose(target1.x3, target1.y3, 0.224, 0, 1, 0, 0);

    // 执行 ManipulatorAction

    manipulator_action.executeAction_trajectory(object1, place_pose1, grasp_pose,"","processing12");
    manipulator_action.executeAction_trajectory(object2, place_pose2, grasp_pose,"processing21","processing22");
    manipulator_action.executeAction_trajectory(object3, place_pose3, grasp_pose,"processing31","processing32");

    rclcpp::shutdown();
    return 0;
}
