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
    auto node_communication = std::make_shared<Communication>();
    // 创建一个单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;// 添加节点到执行器中，以便使其可以处理回调
    executor.add_node(move_group_node);// 在一个新线程中启动执行器的事件循环，以便节点可以异步处理事件。
    executor.add_node(node_communication);

    std::thread([&]() { executor.spin(); }).detach();

    //实例化
    static const std::string PLANNING_GROUP = "arm";
    static const std::string GRIPPER_GROUP = "hand";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects1,collision_objects3,collision_objects2,collision_objects1_1,collision_objects1_2;
    move_group.setMaxVelocityScalingFactor(1);  // 增加速度到 1 倍

    // 创建 Object, TargetPose, GraspPose 实例
    Object object1(move_group_node, "cylinder1");
    Object object2(move_group_node, "cylinder2");
    Object object3(move_group_node, "cylinder3");
    TargetPose place_pose1(move_group_node, move_group, "place_pose1");
    TargetPose place_pose2(move_group_node, move_group, "place_pose2");
    TargetPose place_pose3(move_group_node, move_group, "place_pose3");
    GraspPose grasp_pose(move_group_node, "grasp1");

    position target1,target2,target3;
    // 调用类中的阻塞方法等待消息
    node_communication->waitForMessage();

    // 获取接收到的目标位置信息并赋值给 target1
    node_communication->getPosition(target1);

    object1.setPosition(target1.x1,target1.y1,0.115);
    object2.setPosition(target1.x2,target1.y2,0.115);
    object3.setPosition(target1.x3,target1.y3,0.115);
    // 获取 CollisionObject 并添加到场景中
    collision_objects1.push_back(object1.getCollisionObject());
    collision_objects1.push_back(object2.getCollisionObject());
    collision_objects1.push_back(object3.getCollisionObject());
    // 批量添加物体到场景中
    planning_scene_interface.addCollisionObjects(collision_objects1);

    // 执行 ManipulatorAction
    ManipulatorAction manipulator_action(move_group_node, move_group, planning_scene_interface, gripper_group);
    manipulator_action.executeAction_trajectory(object1, place_pose1, grasp_pose,"","","","");
    //manipulator_action.executeAction_trajectory(object1, place_pose1, grasp_pose,"","","","");

    std::vector<std::string> objects_to_remove1 = {object2.getName(), object3.getName()};
    planning_scene_interface.removeCollisionObjects(objects_to_remove1);

    node_communication->waitForMessage();
    node_communication->getPosition(target2);
    Object object21(move_group_node, "cylinder2");
    Object object22(move_group_node, "cylinder3");

    object21.setPosition(target2.x2,target2.y2,0.115);
    object22.setPosition(target2.x3,target2.y3,0.115);

    collision_objects2.push_back(object21.getCollisionObject());
    collision_objects2.push_back(object22.getCollisionObject());
    // 批量添加物体到场景中
    planning_scene_interface.addCollisionObjects(collision_objects2);
    manipulator_action.executeAction_trajectory(object21, place_pose2, grasp_pose,"","","","");
    //manipulator_action.executeAction_trajectory(object21, place_pose2, grasp_pose,"","","","");
    std::vector<std::string> objects_to_remove2 = {object22.getName()};
    planning_scene_interface.removeCollisionObjects(objects_to_remove2);

    node_communication->waitForMessage();
    node_communication->getPosition(target3);
    Object object31(move_group_node, "cylinder3");


    object31.setPosition(target3.x3,target3.y3,0.115);

    collision_objects3.push_back(object31.getCollisionObject());
    // 批量添加物体到场景中
    planning_scene_interface.addCollisionObjects(collision_objects3);
    manipulator_action.executeAction_trajectory(object31, place_pose3, grasp_pose,"","","","");
    //manipulator_action.executeAction_trajectory(object31, place_pose3, grasp_pose,"","","","");

    std::vector<std::string> objects_to_remove1_1 = {object1.getName(),object21.getName(), object31.getName()};
    planning_scene_interface.removeCollisionObjects(objects_to_remove1_1);

    Object object1_1(move_group_node, "cylinder1");
    Object object2_1(move_group_node, "cylinder2");
    Object object3_1(move_group_node, "cylinder3");

    object1_1.setPositionFromYaml("pose1");
    object2_1.setPositionFromYaml("pose2");
    object3_1.setPositionFromYaml("pose3");

    collision_objects1_1.push_back(object1_1.getCollisionObject());
    collision_objects1_1.push_back(object2_1.getCollisionObject());
    collision_objects1_1.push_back(object3_1.getCollisionObject());
    // 批量添加物体到场景中
    planning_scene_interface.addCollisionObjects(collision_objects1_1);

    manipulator_action.moveAction(object1_1,grasp_pose,"");
    //manipulator_action.moveAction(object1,grasp_pose,"");
    node_communication->waitForMessage();
    node_communication->getPosition(target1);

    place_pose1.setTargetPose(target1.x1, target1.y1, 0.224, 0, 1, 0, 0);
    place_pose2.setTargetPose(target1.x2, target1.y2, 0.224, 0, 1, 0, 0);
    place_pose3.setTargetPose(target1.x3, target1.y3, 0.224, 0, 1, 0, 0);

    manipulator_action.executeAction_trajectory(object1_1, place_pose1, grasp_pose,"","");
    manipulator_action.executeAction_trajectory(object2_1, place_pose2, grasp_pose,"","");
    manipulator_action.executeAction_trajectory(object3_1, place_pose3, grasp_pose,"","");

    // manipulator_action.executeAction_trajectory(object1, place_pose1, grasp_pose,"","");
    // manipulator_action.executeAction_trajectory(object21, place_pose2, grasp_pose,"","");
    // manipulator_action.executeAction_trajectory(object31, place_pose3, grasp_pose,"","");

    place_pose1.setTargetPose(0, -0.1994, 0.42, 0, 1, 0, 0);
    place_pose2.setTargetPose(0.09, -0.1994, 0.42, 0, 1, 0, 0);
    place_pose3.setTargetPose(0.108, -0.1094, 0.42, 0, 1, 0, 0);

    manipulator_action.executeAction_trajectory(object1_1, place_pose1, grasp_pose,"","","","");
    manipulator_action.executeAction_trajectory(object2_1, place_pose2, grasp_pose,"","","","");
    manipulator_action.executeAction_trajectory(object3_1, place_pose3, grasp_pose,"","","","");

    std::vector<std::string> objects_to_remove1_2 = {object1_1.getName(),object2_1.getName(), object3_1.getName()};
    planning_scene_interface.removeCollisionObjects(objects_to_remove1_2);

    Object object1_2(move_group_node, "cylinder1");
    Object object2_2(move_group_node, "cylinder2");
    Object object3_2(move_group_node, "cylinder3");

    object1_2.setPositionFromYaml("pose1");
    object2_2.setPositionFromYaml("pose2");
    object3_2.setPositionFromYaml("pose3");

    collision_objects1_2.push_back(object1_2.getCollisionObject());
    collision_objects1_2.push_back(object2_2.getCollisionObject());
    collision_objects1_2.push_back(object3_2.getCollisionObject());
    // 批量添加物体到场景中
    planning_scene_interface.addCollisionObjects(collision_objects1_2);
    manipulator_action.moveAction(object1_2,grasp_pose,"");
    node_communication->waitForMessage();
    node_communication->getPosition(target1);

    place_pose1.setTargetPose(target1.x1, target1.y1, 0.224, 0, 1, 0, 0);
    place_pose2.setTargetPose(target1.x2, target1.y2, 0.224, 0, 1, 0, 0);
    place_pose3.setTargetPose(target1.x3, target1.y3, 0.224, 0, 1, 0, 0);
    // 执行 ManipulatorAction
    manipulator_action.executeAction_trajectory(object1_2, place_pose1, grasp_pose,"","");
    manipulator_action.executeAction_trajectory(object2_2, place_pose2, grasp_pose,"","");
    manipulator_action.executeAction_trajectory(object3_2, place_pose3, grasp_pose,"","");
    rclcpp::shutdown();
    return 0;
}
