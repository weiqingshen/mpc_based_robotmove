#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.hpp>  // 更新为推荐的头文件
#include <memory>
#include <limits> // 用于检测NaN值
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>  // Eigen库可以帮助你处理三维旋转和方向

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

bool is_valid_pose(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // 检查x, y, z是否为正确范围内的值
    if (std::isnan(msg->position.x) || std::isnan(msg->position.y) || std::isnan(msg->position.z)) {
        RCLCPP_ERROR(LOGGER, "Invalid pose: NaN value detected in x, y, or z.");
        return false;
    }
    // 可以添加更多的检测逻辑
    return true;
}

bool try_plan_and_execute(moveit::planning_interface::MoveGroupInterface& move_group,
                          const geometry_msgs::msg::Pose& target_pose,
                          int max_attempts = 20) {
    bool success = false;  // 记录是否成功
    int attempt_count = 0; // 记录尝试次数

    // 设置目标位姿
    move_group.setPoseTarget(target_pose);

    // 尝试最多max_attempts次规划和执行
    while (attempt_count < max_attempts && !success) {
        // 进行规划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool plan_success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);  // 判断规划是否成功

        if (plan_success) {
            RCLCPP_INFO(LOGGER, "规划成功，尝试执行动作");

            // 尝试执行计划
            moveit::core::MoveItErrorCode execute_result = move_group.execute(my_plan);

            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(LOGGER, "计划执行成功");
                success = true;  // 规划和执行都成功
            } else {
                // 执行失败，尝试重新规划和执行
                attempt_count++;
                RCLCPP_WARN(LOGGER, "计划执行失败，尝试第%d次", attempt_count);
            }
        } else {
            // 规划失败，增加尝试次数
            attempt_count++;
            RCLCPP_WARN(LOGGER, "规划失败，尝试第%d次", attempt_count);
        }
    }

    // 如果尝试了max_attempts次后仍然失败，记录错误日志
    if (!success) {
        RCLCPP_ERROR(LOGGER, "规划和执行失败，尝试了%d次后仍失败", max_attempts);
    }

    return success;  // 返回最终是否成功
}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);// 初始化ROS2节点系统，使系统可以处理ROS2的参数和通信功能
    rclcpp::NodeOptions node_options;//
    node_options.automatically_declare_parameters_from_overrides(true);

    // 创建节点 命名为move_group_interface
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

    // 创建一个单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;// 添加节点到执行器中，以便使其可以处理回调
    executor.add_node(move_group_node);// 在一个新线程中启动执行器的事件循环，以便节点可以异步处理事件。
    std::thread([&]() { executor.spin(); }).detach();

    //实例化
    static const std::string PLANNING_GROUP = "arm";
    static const std::string GRIPPER_GROUP = "hand";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlannerId("BFMTkConfigDefault");  // 设置为BFMT算法
    move_group.setMaxVelocityScalingFactor(1);  // 增加速度到 1 倍

    gripper_group.setNamedTarget("open");
    gripper_group.move();

    // 获取当前机器人状态，并从中获取关于"robot_arm"规划组的信息。
    const moveit::core::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // 创建一个向下方向1
    Eigen::Vector3d axis1(0.0, 1.0, 0.0);  // 指定沿Y轴的旋转
    Eigen::Quaterniond quaternion1(Eigen::AngleAxisd(M_PI, axis1));
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = quaternion1.x();
    target_pose1.orientation.y = quaternion1.y();
    target_pose1.orientation.z = quaternion1.z();
    target_pose1.orientation.w = quaternion1.w();

    bool received_valid_pose = false; // 标志位，表示是否接收到有效的目标位置

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose>> pose_subscriber;// 创建订阅者
    pose_subscriber = move_group_node->create_subscription<geometry_msgs::msg::Pose>(
            "output_topic", 10,
            [&target_pose1, &received_valid_pose, &planning_scene_interface, &pose_subscriber](const geometry_msgs::msg::Pose::SharedPtr msg) {
                // 检查消息是否为有效的格式
                if (!is_valid_pose(msg)) {
                    return; // 如果格式无效，不做任何处理
                }

                // 如果已经接收到有效消息，忽略后续的消息
                if (received_valid_pose) {
                    return; // 已经处理过，不再重复处理
                }

                // 更新 target_pose1 的位置
                target_pose1.position.x = msg->position.x;
                target_pose1.position.y = msg->position.y;
                target_pose1.position.z = msg->position.z;

                RCLCPP_INFO(LOGGER, "Updated target_pose1: x=%f, y=%f, z=%f",
                            target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);

                received_valid_pose = true; // 设置标志位表示接收到有效的目标位置

                // 取消订阅以避免接收重复消息
                pose_subscriber.reset(); // 调用 reset() 取消订阅
            });

    // 等待接收有效的目标位置
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    while (!received_valid_pose) {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_INFO(LOGGER, "等待发布位置");
    }

    std::vector<std::string> object_ids = {"cylinder_1", "cylinder_2", "cylinder_3"};
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    // 创建三个圆柱体并添加到collision_objects
    for (int i = 0; i < 3; ++i) {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "world";  // 使用世界坐标系作为物体初始位置的参考

        collision_object.id = object_ids[i];

        shape_msgs::msg::SolidPrimitive cylinder_primitive;
        cylinder_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        cylinder_primitive.dimensions.resize(2);
        cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.075;
        cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.025;


        geometry_msgs::msg::Pose object_pose;
        object_pose.orientation.w = 1.0;
        object_pose.position.x = target_pose1.position.x + i * 0.15;  // 每个物体的x坐标间隔0.1
        object_pose.position.y = target_pose1.position.y;
        object_pose.position.z = target_pose1.position.z - 0.16;  // 确保物体位于手爪下方

        collision_object.primitives.push_back(cylinder_primitive);
        collision_object.primitive_poses.push_back(object_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    planning_scene_interface.addCollisionObjects(collision_objects);

    // 执行抓取规划
    if (!try_plan_and_execute(move_group, target_pose1)) {
        gripper_group.detachObject(collision_objects[0].id);
        gripper_group.detachObject(collision_objects[1].id);
        gripper_group.detachObject(collision_objects[2].id);
        planning_scene_interface.removeCollisionObjects(object_ids);
        rclcpp::shutdown();  // 如果规划失败，退出程序
        return 1;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    gripper_group.setNamedTarget("close");
    gripper_group.move();  // 执行手爪关闭动作

    rclcpp::sleep_for(std::chrono::seconds(1));

    std::vector<std::string> touch_links;
    touch_links.push_back("link_right");
    touch_links.push_back("link_left");
    gripper_group.attachObject(collision_objects[0].id, "link_hand", touch_links);

    move_group.setStartStateToCurrentState();

    geometry_msgs::msg::Pose target_pose2;

    Eigen::Vector3d axis(0.0, 1.0, 0.0);  // 指定沿Y轴的旋转

    // 通过Eigen的AngleAxis来生成四元数, M_PI/2为旋转角度
    Eigen::Quaterniond quaternion(Eigen::AngleAxisd(M_PI, axis));

    target_pose2.orientation.x = quaternion.x();
    target_pose2.orientation.y = quaternion.y();
    target_pose2.orientation.z = quaternion.z();
    target_pose2.orientation.w = quaternion.w();
    target_pose2.position.x = 0.05;
    target_pose2.position.y = -0.18;
    target_pose2.position.z = 0.41;

    // 尝试执行抓取规划
    if (!try_plan_and_execute(move_group, target_pose2)) {
        gripper_group.detachObject(collision_objects[0].id);
        gripper_group.detachObject(collision_objects[1].id);
        gripper_group.detachObject(collision_objects[2].id);
        planning_scene_interface.removeCollisionObjects(object_ids);
        rclcpp::shutdown();  // 如果规划失败，退出程序
        return 1;
    }

    // 执行手爪打开操作，放开第一个物体
    gripper_group.setNamedTarget("open");
    gripper_group.move();  // 执行打开手爪的动作
    gripper_group.detachObject(collision_objects[0].id);

    // 设置第i个物体的位置
    geometry_msgs::msg::Pose target_pose;

    for (int i = 1; i < 3; ++i) {  // 从第二个物体开始
        RCLCPP_INFO(LOGGER, "移动到第 %d 个圆柱体的位置 %s", i + 1, object_ids[i].c_str());

        target_pose.orientation.x = quaternion.x();
        target_pose.orientation.y = quaternion.y();
        target_pose.orientation.z = quaternion.z();
        target_pose.orientation.w = quaternion.w();
        target_pose.position.x = target_pose1.position.x + i * 0.15;  // 每个物体的x坐标间隔0.1
        target_pose.position.y = target_pose1.position.y;
        target_pose.position.z = target_pose1.position.z;  // 保持相同的高度

        // 尝试执行抓取规划
        if (!try_plan_and_execute(move_group, target_pose)) {
            gripper_group.detachObject(collision_objects[0].id);
            gripper_group.detachObject(collision_objects[1].id);
            gripper_group.detachObject(collision_objects[2].id);
            planning_scene_interface.removeCollisionObjects(object_ids);
            rclcpp::shutdown();  // 如果规划失败，退出程序
            return 1;
        }
        // 抓取第i个物体
        RCLCPP_INFO(LOGGER, "抓取第 %d 个圆柱体 %s", i + 1, object_ids[i].c_str());
        // 执行手爪关闭动作以抓住第i个物体
        gripper_group.setNamedTarget("close");
        gripper_group.move();  // 执行关闭手爪的动作
        gripper_group.attachObject(collision_objects[i].id, "link_hand", touch_links);
        // 移动到放置位置
        RCLCPP_INFO(LOGGER, "移动到放置位置");
        // 设置放置位置的目标姿态 (根据实际需要调整)
        geometry_msgs::msg::Pose place_pose;
        place_pose.orientation.x = quaternion.x();
        place_pose.orientation.y = quaternion.y();
        place_pose.orientation.z = quaternion.z();
        place_pose.orientation.w = quaternion.w();
        if(i==1){
            place_pose.position.x = 0.14;
            place_pose.position.y = -0.16;
            place_pose.position.z = 0.41;
        }

        else if(i==2){
            place_pose.position.x = 0.13;
            place_pose.position.y = 0.02;
            place_pose.position.z = 0.33;
        }
        // 尝试执行抓取规划
        if (!try_plan_and_execute(move_group, place_pose)) {
            gripper_group.detachObject(collision_objects[0].id);
            gripper_group.detachObject(collision_objects[1].id);
            gripper_group.detachObject(collision_objects[2].id);
            planning_scene_interface.removeCollisionObjects(object_ids);
            rclcpp::shutdown();  // 如果规划失败，退出程序
            return 1;
        }
        // 执行手爪打开操作，放开第i个物体
        gripper_group.setNamedTarget("open");
        gripper_group.move();  // 执行打开手爪的动作
        gripper_group.detachObject(collision_objects[i].id);
    }

    // 最后移除所有物体
    rclcpp::sleep_for(std::chrono::seconds(1));
    gripper_group.detachObject(collision_objects[0].id);
    gripper_group.detachObject(collision_objects[1].id);
    gripper_group.detachObject(collision_objects[2].id);
    // 然后将 object_ids 传递给 removeCollisionObjects
    planning_scene_interface.removeCollisionObjects(object_ids);
    rclcpp::shutdown();
    return 0;
}