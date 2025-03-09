#include "arm_control.h"
#include <filesystem>
#include <fstream>  // 用于文件操作
#include <sys/stat.h>  // 用于检查文件是否存在

// 构造函数，只需提供目标姿态的名字，从 YAML 文件加载
TargetPose::TargetPose(const rclcpp::Node::SharedPtr& node,
                       moveit::planning_interface::MoveGroupInterface& move_group,
                       const std::string& pose_name)
        : node_(node), move_group_(move_group), pose_name_(pose_name) {
    if (!isloadTargetPose())
        setTargetPose();
}

// 构造函数，直接提供位置信息和四元数来设置目标姿态
TargetPose::TargetPose(const rclcpp::Node::SharedPtr& node,
                       moveit::planning_interface::MoveGroupInterface& move_group,
                       double x, double y, double z, double qx, double qy, double qz, double qw)
        : node_(node), move_group_(move_group) {
    // 使用传入的参数设置目标姿态
    setTargetPose(x, y, z, qx, qy, qz, qw);
}

// 从 YAML 文件加载指定名字的目标姿态
bool TargetPose::isloadTargetPose() {
    try {
        std::string yaml_file_path=getDefaultYamlPath();
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        if (config["poses"] && config["poses"][pose_name_]) {
            return loadPoseFromConfig(config["poses"][pose_name_]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Pose '%s' not found in YAML file.", pose_name_.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load target pose by name: %s", e.what());
        return false;
    }
}

// 从配置中加载目标姿态
bool TargetPose::loadPoseFromConfig(const YAML::Node& config) {
    try {
        target_pose_.position.x = config["position"]["x"].as<double>();
        target_pose_.position.y = config["position"]["y"].as<double>();
        target_pose_.position.z = config["position"]["z"].as<double>();
        target_pose_.orientation.x = config["orientation"]["qx"].as<double>();
        target_pose_.orientation.y = config["orientation"]["qy"].as<double>();
        target_pose_.orientation.z = config["orientation"]["qz"].as<double>();
        target_pose_.orientation.w = config["orientation"]["qw"].as<double>();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load pose from config: %s", e.what());
        return false;
    }
}

// 通过直接传入位置和四元数设置目标姿态
void TargetPose::setTargetPose(double x, double y, double z, double qx, double qy, double qz, double qw) {
    target_pose_.position.x = x;
    target_pose_.position.y = y;
    target_pose_.position.z = z;
    target_pose_.orientation.x = qx;
    target_pose_.orientation.y = qy;
    target_pose_.orientation.z = qz;
    target_pose_.orientation.w = qw;

    // 设置 MoveGroup 的目标位姿
    move_group_.setPoseTarget(target_pose_);
}

// 设置目标姿态（从加载或直接设定）
void TargetPose::setTargetPose() {
    move_group_.setPoseTarget(target_pose_);
}

// 获取当前目标姿态
geometry_msgs::msg::Pose TargetPose::getTargetPose() const {
    return target_pose_;
}
std::string Object::getDefaultYamlPath()const {
    try {
        // 动态获取路径并拼接
        return ament_index_cpp::get_package_share_directory("mybot") + "/config/param.yaml";
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Object"), "Failed to find package path: %s", e.what());
        throw;  // 抛出异常，保证调用者可以处理
    }
}
Object::Object(const rclcpp::Node::SharedPtr& node, const std::string& object_name)
    : node_(node), object_name_(object_name) {

    // 直接使用动态路径获取 YAML 文件路径
    yaml_file_path_ = getDefaultYamlPath();  // 使用动态路径获取 YAML 文件

    try {
        // 尝试加载 YAML 文件
        YAML::Node config = YAML::LoadFile(yaml_file_path_);

        // 检查 YAML 文件是否成功加载
        if (!config) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", yaml_file_path_.c_str());
            return;
        }

        // 检查 "obstacles" 部分是否存在
        if (!config["obstacles"]) {
            RCLCPP_ERROR(node_->get_logger(), "'obstacles' section not found in YAML file.");
            return;
        }

        // 检查是否存在指定的 object_name
        if (config["obstacles"][object_name_]) {
            RCLCPP_INFO(node_->get_logger(), "Object '%s' found in YAML file.", object_name_.c_str());

            // 调用 loadShapeFromConfig 来加载形状
            if (!loadShapeFromConfig(config["obstacles"][object_name_])) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to load shape configuration for object '%s'.", object_name_.c_str());
            } else {
                RCLCPP_INFO(node_->get_logger(), "Shape configuration for object '%s' loaded successfully.", object_name_.c_str());
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Object '%s' not found in 'obstacles' section of YAML file.", object_name_.c_str());
        }
    } catch (const std::exception& e) {
        // 捕捉并记录异常
        RCLCPP_ERROR(node_->get_logger(), "Exception occurred while loading YAML file: %s", e.what());
    }

    // 初始化姿态（如果加载形状成功，姿态可能已由 config 设置）
    pose_.orientation.w = 1.0; // 默认姿态
}

// 从 YAML 配置中加载形状
bool Object::loadShapeFromConfig(const YAML::Node& config) {
    try {
        std::string type = config["type"].as<std::string>();

        if (type == "box") {
            primitive_.type = primitive_.BOX;
            primitive_.dimensions = {
                    config["dimensions"][0].as<double>(),
                    config["dimensions"][1].as<double>(),
                    config["dimensions"][2].as<double>()
            };
        } else if (type == "cylinder") {
            primitive_.type = primitive_.CYLINDER;
            primitive_.dimensions = {
                    config["dimensions"]["height"].as<double>(), // 高度
                    config["dimensions"]["radius"].as<double>()  // 半径
            };
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown shape type '%s' for object '%s'.", type.c_str(), object_name_.c_str());
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load shape from config: %s", e.what());
        return false;
    }
}

// 设置物体的位置
void Object::setPosition(float x,float y,float z) {

    pose_.position.x = x;
    pose_.position.y = y;
    pose_.position.z = z;
}

void Object::setPositionFromYaml(const std::string& pose_name) {
    try {
        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(yaml_file_path_);
        if (!config) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", yaml_file_path_.c_str());
            return;
        }

        // 检查 "poses" 部分是否存在
        if (!config["poses"]) {
            RCLCPP_ERROR(node_->get_logger(), "'poses' section not found in YAML file.");
            return;
        }

        // 检查是否存在指定的 pose_name
        if (!config["poses"][pose_name]) {
            RCLCPP_ERROR(node_->get_logger(), "Pose '%s' not found in 'poses' section of YAML file.", pose_name.c_str());
            return;
        }

        // 提取位置信息
        YAML::Node pose_config = config["poses"][pose_name]["position"];
        if (pose_config) {
            pose_.position.x = pose_config["x"].as<float>();
            pose_.position.y = pose_config["y"].as<float>();
            pose_.position.z = pose_config["z"].as<float>();
            RCLCPP_INFO(node_->get_logger(), "Position for pose '%s' set to [x: %f, y: %f, z: %f].",
                        pose_name.c_str(), pose_.position.x, pose_.position.y, pose_.position.z);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "'position' section not found for pose '%s' in YAML file.", pose_name.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception occurred while setting position from YAML file: %s", e.what());
    }
}

std::string GraspPose::getDefaultYamlPath() const{
    try {
        // 动态获取路径并拼接
        return ament_index_cpp::get_package_share_directory("mybot") + "/config/param.yaml";
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Object"), "Failed to find package path: %s", e.what());
        throw;  // 抛出异常，保证调用者可以处理
    }
}

GraspPose::GraspPose(const rclcpp::Node::SharedPtr& node, const std::string& grasp_name)
        : node_(node), grasp_name_(grasp_name) {
    try {
        // 使用 getDefaultYamlPath() 获取默认的 YAML 文件路径
        std::string yaml_file_path = getDefaultYamlPath();
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        if (config["grasp_poses"] && config["grasp_poses"][grasp_name]) {
            loadGraspOffsetFromConfig(config["grasp_poses"][grasp_name]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Grasp pose '%s' not found in YAML file.", grasp_name_.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load grasp pose for '%s': %s", grasp_name_.c_str(), e.what());
    }
}

// 从 YAML 配置中加载抓取偏移
bool GraspPose::loadGraspOffsetFromConfig(const YAML::Node& config) {
    try {
        offset_pose_.position.x = config["offset"]["x"].as<double>();
        offset_pose_.position.y = config["offset"]["y"].as<double>();
        offset_pose_.position.z = config["offset"]["z"].as<double>();
        offset_pose_.orientation.x = config["orientation"]["qx"].as<double>();
        offset_pose_.orientation.y = config["orientation"]["qy"].as<double>();
        offset_pose_.orientation.z = config["orientation"]["qz"].as<double>();
        offset_pose_.orientation.w = config["orientation"]["qw"].as<double>();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load grasp offset from config: %s", e.what());
        return false;
    }
}


// 获取抓取偏移姿态
geometry_msgs::msg::Pose GraspPose::getGraspOffsetPose(const geometry_msgs::msg::Pose& object_pose) const {
    geometry_msgs::msg::Pose grasp_pose = object_pose;

    // 应用偏移
    grasp_pose.position.x += offset_pose_.position.x;
    grasp_pose.position.y += offset_pose_.position.y;
    grasp_pose.position.z += offset_pose_.position.z;

    // 应用抓取方向（忽略实际合并方向）
    grasp_pose.orientation = offset_pose_.orientation;

    return grasp_pose;
}

geometry_msgs::msg::Pose GraspPose::getObjectOffsetPose(const geometry_msgs::msg::Pose &target_pose) const {
    geometry_msgs::msg::Pose adjusted_pose = target_pose;

    // 将偏移添加到目标位置
    adjusted_pose.position.x -= offset_pose_.position.x;
    adjusted_pose.position.y -= offset_pose_.position.y;
    adjusted_pose.position.z -= offset_pose_.position.z;

    // 如果需要，你可以根据应用的逻辑调整方向（四元数）。
    // 这里暂时假设目标姿态的方向完全由偏移姿态来决定
    adjusted_pose.orientation = offset_pose_.orientation;

    return adjusted_pose;
}


// 获取物体的碰撞对象
moveit_msgs::msg::CollisionObject Object::getCollisionObject() const {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = object_name_;
    collision_object.header.frame_id = "world"; // 根据实际需要修改
    collision_object.primitives.push_back(primitive_);
    collision_object.primitive_poses.push_back(pose_);
    collision_object.operation = collision_object.ADD;
    return collision_object;
}

// 将物体添加到规划场景中
void Object::addToPlanningScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    planning_scene_interface.applyCollisionObject(getCollisionObject());
}
std::string Object::getName() const {
    return object_name_; // 返回物体名称
}
void Object::shrink(float scale_factor) {
    // 确保缩放因子大于 0
    if (scale_factor <= 0) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid scale factor: %f", scale_factor);
        return;
    }

    // 检查物体类型，并根据类型调整其尺寸
    if (primitive_.type == shape_msgs::msg::SolidPrimitive::CYLINDER) {
        // 缩小圆柱的尺寸
        primitive_.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] *= scale_factor;
        primitive_.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] *= scale_factor;
    }
    // 如果有其他形状类型，也可以在这里添加处理
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unsupported shape type: %d", primitive_.type);
    }

    // 更新物体的碰撞对象
    RCLCPP_INFO(node_->get_logger(), "Object %s shrunk with scale factor: %f", object_name_.c_str(), scale_factor);
}

// ManipulatorAction 构造函数
ManipulatorAction::ManipulatorAction(const rclcpp::Node::SharedPtr& node,
                                     moveit::planning_interface::MoveGroupInterface& move_group,
                                     moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                     moveit::planning_interface::MoveGroupInterface& gripper_group)
        : node_(node), move_group_(move_group), planning_scene_interface_(planning_scene_interface), gripper_group_(gripper_group),
          trajectory_(node,"default_trajectory.yaml") {  // 只传递 YAML 文件
    // 其他初始化代码...
    // 创建发布器，用于发送轨迹执行状态消息
    trajectory_status_publisher_ = node_->create_publisher<std_msgs::msg::String>("trajectory_status", 100);
    trajectory_pub_ = node->create_publisher<moveit_msgs::msg::RobotTrajectory>("manipulator_trajectory", 100);
    grasp_publisher_ = node->create_publisher<std_msgs::msg::Float32>("gripper_control", 100);
}


void ManipulatorAction::executeAction(Object& object, TargetPose& target_pose, GraspPose& grasp_pose, const std::string& trajectory_name) {
    // 指定差值数量
    size_t num_interpolation_points = 0;

    geometry_msgs::msg::Pose target_initial_pose;
    target_initial_pose.position.x = 0.0363116;  // 替换为目标x坐标
    target_initial_pose.position.y = 0.204015;  // 替换为目标y坐标
    target_initial_pose.position.z = 0.401;  // 替换为目标z坐标
    target_initial_pose.orientation.x = 0.0;  // 替换为四元数x
    target_initial_pose.orientation.y = 1.0;  // 替换为四元数y
    target_initial_pose.orientation.z = 0.0;  // 替换为四元数z
    target_initial_pose.orientation.w = 0.0;  // 替换为四元数w

    // 1. 获取抓取偏移姿态并移动到物体的位置
    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
    moveToPose_trajectory(grasp_offset_pose);

    // 2. 闭合夹爪
    closeGripper();

    // 3. Attach 物体
    attachObject(object);

    moveToPose_trajectory(target_initial_pose);

    // 打开夹爪
    openGripper();
    // 闭合夹爪
    closeGripper();
    //这里是作为分割轨迹用 正常可以不需要

    //move_group_.setNamedTarget("initial");
    //if(move_group_.move()) {
    //    RCLCPP_INFO(node_->get_logger(),"规划成功");
    //}
    //else {
    //    RCLCPP_INFO(node_->get_logger(),"回到初始位置失败");
    //}

// 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name.c_str());
        trajectory_.executeTrajectory(move_group_, trajectory_name);
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        // 4. 移动到目标位置
        moveToPose_trajectory(target_pose.getTargetPose());
    }

    // 5. 打开夹爪
    openGripper();

    // 6. Detach 物体
    detachObject(object);

    // 7. 使用 TargetPose 的目标位置来更新物体的坐标
    geometry_msgs::msg::Pose final_pose = grasp_pose.getObjectOffsetPose(target_pose.getTargetPose());

    // 更新物体的坐标
    object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "物体坐标已更新到 (%f, %f, %f)", final_pose.position.x, final_pose.position.y, final_pose.position.z);

    moveToPose_trajectory(target_initial_pose);
    closeGripper();
    openGripper();
}
void ManipulatorAction::moveAction(Object& object, GraspPose& grasp_pose, const std::string& trajectory_name) {
    // 1. 获取抓取偏移姿态并移动到物体的位置
    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
    // 发布轨迹开始执行的消息
    std_msgs::msg::String msg;
if (!trajectory_name.empty()) {

    RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name.c_str());
    trajectory_.setYamlFileName(trajectory_name);
    if(!trajectory_.executeTrajectory(move_group_, trajectory_name)) {
        msg.data = trajectory_name + " 开始执行";
        trajectory_status_publisher_->publish(msg);

        moveToPose_trajectory(grasp_offset_pose,false);
        // 发布轨迹执行完成的消息
        msg.data = "执行完成";
        trajectory_status_publisher_->publish(msg);
    }
} else {
    RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
    moveToPose_trajectory(grasp_offset_pose);
    // 4. 移动到目标位置
}

}
void ManipulatorAction::placeAction(Object& object,TargetPose& target_pose, GraspPose& grasp_pose) {
    // 1. 获取抓取偏移姿态并移动到物体的位置

    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(target_pose.getTargetPose());
    moveToPose_trajectory(grasp_offset_pose);
    openGripper();
    detachObject(object);

    geometry_msgs::msg::Pose target_initial_pose;
    target_initial_pose.position.x = 0.0363116;  // 替换为目标x坐标
    target_initial_pose.position.y = 0.204015;  // 替换为目标y坐标
    target_initial_pose.position.z = 0.401;  // 替换为目标z坐标
    target_initial_pose.orientation.x = 0.0;  // 替换为四元数x
    target_initial_pose.orientation.y = 1.0;  // 替换为四元数y
    target_initial_pose.orientation.z = 0.0;  // 替换为四元数z
    target_initial_pose.orientation.w = 0.0;  // 替换为四元数w

    moveToPose_trajectory(target_initial_pose);

    geometry_msgs::msg::Pose final_pose = target_pose.getTargetPose();
    object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
}
// void ManipulatorAction::executeAction_trajectory(Object& object, TargetPose& target_pose,GraspPose& grasp_pose, const std::string& trajectory_name1,const std::string& trajectory_name2) {
//
//     // 1. 获取抓取偏移姿态并移动到物体的位置
//     geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
//     moveToPose(grasp_offset_pose);
//
//     // 2. 闭合夹爪
//     closeGripper();
//
//     // 3. Attach 物体
//     attachObject(object);
//
//     geometry_msgs::msg::Pose target_initial_pose;
//     target_initial_pose.position.x = 0.0363116;  // 替换为目标x坐标
//     target_initial_pose.position.y = 0.204015;  // 替换为目标y坐标
//     target_initial_pose.position.z = 0.401;  // 替换为目标z坐标
//     target_initial_pose.orientation.x = 0.0;  // 替换为四元数x
//     target_initial_pose.orientation.y = 1.0;  // 替换为四元数y
//     target_initial_pose.orientation.z = 0.0;  // 替换为四元数z
//     target_initial_pose.orientation.w = 0.0;  // 替换为四元数w
//
//     moveToPose(target_initial_pose);
//     // 如果传递了轨迹的名字，先执行传递的轨迹
//     if (!trajectory_name1.empty()) {
//         RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name1.c_str());
//         trajectory_.setYamlFileName(trajectory_name1);
//         trajectory_.executeTrajectory(move_group_, trajectory_name1);
//     } else {
//         RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
//         // 4. 移动到目标位置
//         //moveToPose(target_pose.getTargetPose());
//     }
//     moveToPose(target_pose.getTargetPose());//这一行必不可少 如果缺少了 可能无法完成物体的坐标更新
//     // 5. 打开夹爪
//     openGripper();
//
//     // 6. Detach 物体
//     detachObject(object);
//
//     // 如果传递了轨迹的名字，先执行传递的轨迹
//     if (!trajectory_name2.empty()) {
//         RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name2.c_str());
//         trajectory_.setYamlFileName(trajectory_name2);
//         trajectory_.executeTrajectory(move_group_, trajectory_name2);
//     } else {
//         RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
//         // 4. 移动到目标位置
//         //moveToPose(target_pose.getTargetPose());
//     }
//     // 7. 使用 TargetPose 的目标位置来更新物体的坐标
//     geometry_msgs::msg::Pose final_pose = grasp_pose.getObjectOffsetPose(target_pose.getTargetPose());
//
//     // 更新物体的坐标
//     object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
//     RCLCPP_INFO(node_->get_logger(), "物体坐标已更新到 (%f, %f, %f)", final_pose.position.x, final_pose.position.y, final_pose.position.z);
//
//     moveToPose(target_initial_pose);
//
// }


void ManipulatorAction::executeAction_trajectory(Object& object, TargetPose& target_pose,GraspPose& grasp_pose, const std::string& trajectory_name1,const std::string& trajectory_name2,const std::string& trajectory_name3,const std::string& trajectory_name4) {
    geometry_msgs::msg::Pose target_initial_pose;
    target_initial_pose.position.x = 0.0363116;  // 替换为目标x坐标
    target_initial_pose.position.y = 0.204015;  // 替换为目标y坐标
    target_initial_pose.position.z = 0.401;  // 替换为目标z坐标
    target_initial_pose.orientation.x = 0.0;  // 替换为四元数x
    target_initial_pose.orientation.y = 1.0;  // 替换为四元数y
    target_initial_pose.orientation.z = 0.0;  // 替换为四元数z
    target_initial_pose.orientation.w = 0.0;  // 替换为四元数w
    // 发布轨迹开始执行的消息
    std_msgs::msg::String msg;
    // 1. 获取抓取偏移姿态并移动到物体的位置
    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
    if (!trajectory_name1.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name1.c_str());
        trajectory_.setYamlFileName(trajectory_name1);
        // 尝试执行轨迹
        if (!trajectory_.executeTrajectory(move_group_, trajectory_name1)) {
            msg.data = trajectory_name1 + " 开始执行";
            trajectory_status_publisher_->publish(msg);
            RCLCPP_WARN(node_->get_logger(), "轨迹执行失败，尝试直接移动到目标位置");
            moveToPose_trajectory(grasp_offset_pose,false);  // 在失败时仍尝试移动到 grasp_offset_pose
            // 发布轨迹执行完成的消息
            msg.data = "执行完成";
            trajectory_status_publisher_->publish(msg);
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        moveToPose_trajectory(grasp_offset_pose);
        // 4. 移动到目标位置
    }
    //moveToPose(grasp_offset_pose);

    // 2. 闭合夹爪
    closeGripper();

    // 3. Attach 物体
    attachObject(object);
    // 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name2.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name2.c_str());
        trajectory_.setYamlFileName(trajectory_name2);
        if(!trajectory_.executeTrajectory(move_group_, trajectory_name2)) {
            msg.data = trajectory_name2 + " 开始执行";
            trajectory_status_publisher_->publish(msg);
            moveToPose_trajectory(target_initial_pose,false);
            // 发布轨迹执行完成的消息
            msg.data = "执行完成";
            trajectory_status_publisher_->publish(msg);
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        moveToPose_trajectory(target_initial_pose);
        // 4. 移动到目标位置
    }
    // openGripper();
    // closeGripper();
    //moveToPose(target_initial_pose);
    // 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name3.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name3.c_str());
        trajectory_.setYamlFileName(trajectory_name3);
        if(!trajectory_.executeTrajectory(move_group_, trajectory_name3)) {
            msg.data = trajectory_name3 + " 开始执行";
            trajectory_status_publisher_->publish(msg);
            moveToPose_trajectory(target_pose.getTargetPose(),false);
            // 发布轨迹执行完成的消息
            msg.data = "执行完成";
            trajectory_status_publisher_->publish(msg);
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        moveToPose_trajectory(target_pose.getTargetPose());
        // 4. 移动到目标位置
    }
    //moveToPose(target_pose.getTargetPose());//这一行必不可少 如果缺少了 可能无法完成物体的坐标更新
    // 5. 打开夹爪
    openGripper();

    // 6. Detach 物体
    detachObject(object);

    // 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name4.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name4.c_str());
        trajectory_.setYamlFileName(trajectory_name4);
        if(!trajectory_.executeTrajectory(move_group_, trajectory_name4)) {
            msg.data = trajectory_name4 + " 开始执行";
            trajectory_status_publisher_->publish(msg);
            moveToPose_trajectory(target_initial_pose,false);
            // 发布轨迹执行完成的消息
            msg.data = "执行完成";
            trajectory_status_publisher_->publish(msg);
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        moveToPose_trajectory(target_initial_pose);
        // 4. 移动到目标位置
    }
    // closeGripper();
    // openGripper();
    //moveToPose(target_initial_pose);
    // 7. 使用 TargetPose 的目标位置来更新物体的坐标
    geometry_msgs::msg::Pose final_pose = grasp_pose.getObjectOffsetPose(target_pose.getTargetPose());

    // 更新物体的坐标
    object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "物体坐标已更新到 (%f, %f, %f)", final_pose.position.x, final_pose.position.y, final_pose.position.z);
}
void ManipulatorAction::graspAction_trajectory(Object& object, TargetPose& target_pose,GraspPose& grasp_pose, const std::string& trajectory_name1,const std::string& trajectory_name2,const std::string& trajectory_name3,const std::string& trajectory_name4) {
    geometry_msgs::msg::Pose target_initial_pose;
    target_initial_pose.position.x = 0.0363116;  // 替换为目标x坐标
    target_initial_pose.position.y = 0.204015;  // 替换为目标y坐标
    target_initial_pose.position.z = 0.401;  // 替换为目标z坐标
    target_initial_pose.orientation.x = 0.0;  // 替换为四元数x
    target_initial_pose.orientation.y = 1.0;  // 替换为四元数y
    target_initial_pose.orientation.z = 0.0;  // 替换为四元数z
    target_initial_pose.orientation.w = 0.0;  // 替换为四元数w

    // 发布轨迹开始执行的消息
    std_msgs::msg::String msg;
    // 1. 获取抓取偏移姿态并移动到物体的位置
    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
    if (!trajectory_name1.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name1.c_str());
        trajectory_.setYamlFileName(trajectory_name1);
        // 尝试执行轨迹
        if (!trajectory_.executeTrajectory(move_group_, trajectory_name1)) {
            msg.data = trajectory_name1 + " 开始执行";
            trajectory_status_publisher_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name1.c_str());
            trajectory_.setYamlFileName(trajectory_name1);
            moveToPose_trajectory(grasp_offset_pose,false);  // 在失败时仍尝试移动到 grasp_offset_pose
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        moveToPose_trajectory(grasp_offset_pose);
        // 4. 移动到目标位置
    }
    //moveToPose(grasp_offset_pose);

    // 2. 闭合夹爪
    closeGripper();

    // 3. Attach 物体
    attachObject(object);
    // 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name2.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name2.c_str());
        trajectory_.setYamlFileName(trajectory_name2);
        if(!trajectory_.executeTrajectory(move_group_, trajectory_name2)) {
            msg.data = trajectory_name2 + " 开始执行";
            trajectory_status_publisher_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name2.c_str());
            trajectory_.setYamlFileName(trajectory_name2);
            moveToPose_trajectory(target_initial_pose,false);
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");

        moveToPose_trajectory(target_initial_pose);
        // 4. 移动到目标位置
    }
    // openGripper();
    // closeGripper();
    //moveToPose(target_initial_pose);
    // 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name3.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name3.c_str());
        trajectory_.setYamlFileName(trajectory_name3);
        if(!trajectory_.executeTrajectory(move_group_, trajectory_name3)) {
            msg.data = trajectory_name3 + " 开始执行";
            trajectory_status_publisher_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name3.c_str());
            trajectory_.setYamlFileName(trajectory_name3);
            moveToPose_trajectory(target_pose.getTargetPose(),false);
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        moveToPose_trajectory(target_pose.getTargetPose());
        // 4. 移动到目标位置
    }
    //moveToPose(target_pose.getTargetPose());//这一行必不可少 如果缺少了 可能无法完成物体的坐标更新
    // 5. 打开夹爪
    // openGripper();
    //
    // // 6. Detach 物体
    // detachObject(object);

    // 如果传递了轨迹的名字，先执行传递的轨迹
    // if (!trajectory_name4.empty()) {
    //     RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name4.c_str());
    //     trajectory_.setYamlFileName(trajectory_name4);
    //     if(!trajectory_.executeTrajectory(move_group_, trajectory_name4)) {
    //         moveToPose(target_initial_pose);
    //     }
    // } else {
    //     RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
    //     moveToPose(target_initial_pose);
    //     // 4. 移动到目标位置
    // }
    // closeGripper();
    // openGripper();
    //moveToPose(target_initial_pose);
    // 7. 使用 TargetPose 的目标位置来更新物体的坐标
    // geometry_msgs::msg::Pose final_pose = grasp_pose.getObjectOffsetPose(target_pose.getTargetPose());
    //
    // // 更新物体的坐标
    // // object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    // RCLCPP_INFO(node_->get_logger(), "物体坐标已更新到 (%f, %f, %f)", final_pose.position.x, final_pose.position.y, final_pose.position.z);
}




// 检查文件是否存在的函数
bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void ManipulatorAction::moveToPose_trajectory(const geometry_msgs::msg::Pose& pose, bool publish_trajectory) {
    bool success = false;
    int attempt_count = 0;
    const int max_attempts = 100;
    bool has_successful_plan = false;
    int max_gripper_attempts = 20;

    // 设置目标位姿
    move_group_.setPoseTarget(pose);

    // 调整轨迹时间（缩放）
    auto scale_trajectory_time = [](moveit_msgs::msg::RobotTrajectory& input_trajectory, double scale_factor) {
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& joint_trajectories = input_trajectory.joint_trajectory.points;
        for (auto& point : joint_trajectories) {
            double original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            double scaled_time = original_time / scale_factor;
            point.time_from_start.sec = static_cast<int>(scaled_time);
            point.time_from_start.nanosec = static_cast<int>((scaled_time - point.time_from_start.sec) * 1e9);
        }
    };

    // 定义轨迹对象
    moveit_msgs::msg::RobotTrajectory final_trajectory;

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
    std::vector<double> trajectory_durations;

    // 尝试规划和执行，最多尝试 max_attempts 次
    while (attempt_count < max_attempts && !success) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // 尝试规划
        bool plan_success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_success) {
            RCLCPP_INFO(rclcpp::get_logger("manipulator_action"), "Plan successful, trying to execute");

            // 缩放轨迹时间
            double scale_factor = 7;
            scale_trajectory_time(my_plan.trajectory_, scale_factor);

            // 记录轨迹和时长
            plans.push_back(my_plan);
            trajectory_durations.push_back(my_plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                                           my_plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9);

            // 如果已有成功的路径，并且已达到 10 次规划，则执行时间最短的轨迹
            if (has_successful_plan && attempt_count >= 10) {
                size_t min_duration_index = std::min_element(trajectory_durations.begin(), trajectory_durations.end()) - trajectory_durations.begin();
                RCLCPP_INFO(rclcpp::get_logger("manipulator_action"), "Choosing shortest trajectory to execute");

                // 执行最短时间的轨迹
                moveit::core::MoveItErrorCode execute_result = move_group_.execute(plans[min_duration_index]);

                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(rclcpp::get_logger("manipulator_action"), "Plan executed successfully");
                    final_trajectory = plans[min_duration_index].trajectory_;  // 赋值给最终轨迹
                    success = true;
                } else {
                    attempt_count++;
                    RCLCPP_WARN(rclcpp::get_logger("manipulator_action"), "Plan execution failed, attempt %d", attempt_count);
                }

                break;
            }

            has_successful_plan = true;

            // 如果已记录 5 个轨迹，选择时间最短的一个
            if (plans.size() >= 5) {
                size_t min_duration_index = std::min_element(trajectory_durations.begin(), trajectory_durations.end()) - trajectory_durations.begin();
                RCLCPP_INFO(rclcpp::get_logger("manipulator_action"), "Choosing shortest trajectory to execute");

                moveit::core::MoveItErrorCode execute_result = move_group_.execute(plans[min_duration_index]);

                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(rclcpp::get_logger("manipulator_action"), "Plan executed successfully");
                    final_trajectory = plans[min_duration_index].trajectory_;  // 赋值给最终轨迹
                    success = true;
                } else {
                    attempt_count++;
                    RCLCPP_WARN(rclcpp::get_logger("manipulator_action"), "Plan execution failed, attempt %d", attempt_count);
                }

                break;
            }
        } else {
            attempt_count++;
            RCLCPP_WARN(rclcpp::get_logger("manipulator_action"), "Planning failed, adjusting and retrying, attempt %d", attempt_count);

            // 规划失败时调整四元数，确保 qx 和 qy 的平方和为 1
            geometry_msgs::msg::Pose modified_pose = pose;

            // 进行小角度旋转，避免失败
            double angle = 0.1;
            double cos_half_angle = cos(angle / 2);
            double sin_half_angle = sin(angle / 2);

            modified_pose.orientation.x = cos_half_angle;
            modified_pose.orientation.y = sin_half_angle;
            modified_pose.orientation.z = 0.0;
            modified_pose.orientation.w = 0.0;

            move_group_.setPoseTarget(modified_pose);
        }

        // 获取机器人状态并处理抓取器
        auto robot_state = gripper_group_.getCurrentState();
        if (!robot_state) {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "无法获取机器人当前状态");
            return;
        }

        // 获取 "hand" 组的关节状态
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions("hand", joint_values);

        // 确保获取到关节值
        if (joint_values.empty()) {
            // RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "未能获取 hand 组的关节状态");
            return;
        }

        // 假设 gripper_group_ 中的第一个关节是 "joint_right" 关节，检查其值是否等于 0.04
        double gripper_position = joint_values[0]; // 根据实际的关节顺序调整索引
        std::cout << gripper_position << std::endl;

        // 如果抓取次数已达到最大值且当前抓取器状态是 open，则调用 gripper.openmax()
        if (attempt_count >= max_gripper_attempts && abs(gripper_position - 0.005) < 0.001) {

            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "抓取尝试次数达到最大值，调用 gripper.openmax()");

            // 调用 gripper.openmax() 并继续规划
            openGripper_max();

            // 重新设置目标位姿，并生成新的规划
            move_group_.setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan new_plan;
            if (move_group_.plan(new_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                // RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "重新规划成功");
                final_trajectory = new_plan.trajectory_;  // 赋值给最终轨迹
                move_group_.execute(new_plan);
            } else {
                // RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "重新规划失败");
            }
        }
    }

    // 如果尝试 max_attempts 次后仍然失败，记录错误
    if (!success) {
        // RCLCPP_ERROR(rclcpp::get_logger("manipulator_action"), "Planning and execution failed after %d attempts", max_attempts);
        std::vector<double> joint_values(6, 0.0);
        move_group_.setJointValueTarget(joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan reset_plan;
        if (move_group_.plan(reset_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            // RCLCPP_INFO(rclcpp::get_logger("manipulator_action"), "Plan successful, trying to return to zero position");
            final_trajectory = reset_plan.trajectory_;  // 赋值给最终轨迹
            move_group_.execute(reset_plan);
        } else {
            // RCLCPP_ERROR(rclcpp::get_logger("manipulator_action"), "Returning to zero position failed");
        }
    }

    // 检查 final_trajectory 是否为空
    if (final_trajectory.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("manipulator_action"), "Final trajectory is empty, no trajectory generated");
    } else {
        // 如果 final_trajectory 不为空，则发布实际执行的轨迹
        if (publish_trajectory) {
            RCLCPP_INFO(rclcpp::get_logger("manipulator_action"), "Publishing final trajectory");
            trajectory_pub_->publish(final_trajectory);
        }
    }

}





void ManipulatorAction::moveToPose(const geometry_msgs::msg::Pose& pose) {
    bool success = false;  // 记录是否成功
    int attempt_count = 0;  // 尝试次数
    const int max_attempts = 100;  // 最大尝试次数
    bool has_successful_plan = false;  // 记录是否已有成功的规划路径
    int max_gripper_attempts=20;
    // 设置目标位姿
    move_group_.setPoseTarget(pose);

    // 调整轨迹时间的函数，按比例缩放时间
    auto scale_trajectory_time = [](moveit_msgs::msg::RobotTrajectory& input_trajectory, double scale_factor) {
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& joint_trajectories = input_trajectory.joint_trajectory.points;
        for (auto& point : joint_trajectories) {
            double original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            double scaled_time = original_time / scale_factor;
            point.time_from_start.sec = static_cast<int>(scaled_time);
            point.time_from_start.nanosec = static_cast<int>((scaled_time - point.time_from_start.sec) * 1e9);
        }
    };

    // 存储多个轨迹
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
    std::vector<double> trajectory_durations;


    // 尝试最多 max_attempts 次规划和执行
    while (attempt_count < max_attempts && !success) {
        // 创建一个计划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // 尝试规划
        bool plan_success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_success) {
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "规划成功，尝试执行动作");

            // 按比例缩放轨迹时间
            double scale_factor = 7; // 设置缩放比例
            scale_trajectory_time(my_plan.trajectory_, scale_factor);

            // 记录轨迹和轨迹时长
            plans.push_back(my_plan);
            trajectory_durations.push_back(my_plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                                           my_plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9);

            // 如果已有成功的路径，且已达到10次规划，直接选择最短轨迹
            if (has_successful_plan && attempt_count >= 10) {
                size_t min_duration_index = std::min_element(trajectory_durations.begin(), trajectory_durations.end()) - trajectory_durations.begin();

                RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "选择时间最短的轨迹进行执行");

                // 执行最短时间的轨迹
                moveit::core::MoveItErrorCode execute_result = move_group_.execute(plans[min_duration_index]);

                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "计划执行成功");
                    success = true;  // 规划和执行都成功
                } else {
                    attempt_count++;
                    RCLCPP_WARN(rclcpp::get_logger("move_group_interface"), "计划执行失败，尝试第%d次", attempt_count);
                }

                break; // 一旦执行了最短时间轨迹，跳出循环
            }

            // 标记已成功的规划路径
            has_successful_plan = true;

            // 如果已记录5个轨迹，选择时间点最小的一个
            if (plans.size() >= 5) {
                // 选择时间最短的轨迹
                size_t min_duration_index = std::min_element(trajectory_durations.begin(), trajectory_durations.end()) - trajectory_durations.begin();

                RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "选择时间最短的轨迹进行执行");

                // 执行最短时间的轨迹
                moveit::core::MoveItErrorCode execute_result = move_group_.execute(plans[min_duration_index]);

                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "计划执行成功");
                    success = true;  // 规划和执行都成功
                } else {
                    // 执行失败，增加尝试次数
                    attempt_count++;
                    RCLCPP_WARN(rclcpp::get_logger("move_group_interface"), "计划执行失败，尝试第%d次", attempt_count);
                }

                break; // 一旦执行了最短时间轨迹，跳出循环
            }
        } else {
            // 规划失败，增加尝试次数
            attempt_count++;
            RCLCPP_WARN(rclcpp::get_logger("move_group_interface"), "规划失败，调整角度重新规划，尝试第%d次", attempt_count);

            // 规划失败时调整四元数，确保 qx 和 qy 的平方和为 1
            geometry_msgs::msg::Pose modified_pose = pose;

            // 计算绕Z轴的小角度旋转，例如0.1度（弧度）
            double angle = 0.1; // 旋转角度，单位为弧度
            double cos_half_angle = cos(angle / 2);
            double sin_half_angle = sin(angle / 2);

            // 确保 qx 和 qy 的平方和为1
            modified_pose.orientation.x = cos_half_angle;
            modified_pose.orientation.y = sin_half_angle;
            modified_pose.orientation.z = 0.0;
            modified_pose.orientation.w = 0.0; // qz 和 qw 保持不变

            // 设置修改后的目标位姿
            move_group_.setPoseTarget(modified_pose);
        }


        auto robot_state = gripper_group_.getCurrentState();

        if (!robot_state) {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "无法获取机器人当前状态");
            return;
        }

        // 获取 "hand" 组的关节状态
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions("hand", joint_values);

        // 确保获取到关节值
        if (joint_values.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "未能获取 hand 组的关节状态");
            return;
        }

        // 假设 gripper_group_ 中的第一个关节是 "joint_right" 关节，检查其值是否等于 0.04
        double gripper_position = joint_values[0]; // 根据实际的关节顺序调整索引
        std::cout << gripper_position<<std::endl;

        // 如果抓取次数已达到最大值且当前抓取器状态是 open，则调用 gripper.openmax()
        if (attempt_count >= max_gripper_attempts&&abs(gripper_position-0.005)<0.001) {

            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "抓取尝试次数达到最大值，调用 gripper.openmax()");

            // 调用 gripper.openmax() 并继续规划
            openGripper_max();

            // 重新设置目标位姿，并生成新的规划
            move_group_.setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan new_plan;
            if (move_group_.plan(new_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "重新规划成功");
                move_group_.execute(new_plan);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "重新规划失败");
            }

        }
    }


    // 如果尝试了 max_attempts 次后仍然失败，记录错误日志
    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "规划和执行失败，尝试了%d次后仍然失败", max_attempts);

        // 设置所有关节为0
        std::vector<double> joint_values(6, 0.0); // 机器人有6个关节
        move_group_.setJointValueTarget(joint_values);

        // 创建一个新的计划来让机器人回到初始位置
        moveit::planning_interface::MoveGroupInterface::Plan reset_plan;
        if (move_group_.plan(reset_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "规划成功，尝试返回零位置");
            move_group_.execute(reset_plan);  // 执行返回零位置的动作
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "返回零位置失败");
        }
    }
}




// 闭合夹爪
void ManipulatorAction::closeGripper() {
    RCLCPP_INFO(node_->get_logger(), "Closing gripper.");
    gripper_group_.setNamedTarget("close");
    gripper_group_.move();
    // 发布夹爪闭合的状态
    std_msgs::msg::Float32 msg;
    msg.data = 0.0;  // 夹爪闭合状态，发布 0
    grasp_publisher_->publish(msg);
}

// 打开夹爪
void ManipulatorAction::openGripper() {
    RCLCPP_INFO(node_->get_logger(), "Opening gripper.");
    gripper_group_.setNamedTarget("open");
    gripper_group_.move();
    //发布夹爪打开的状态
    std_msgs::msg::Float32 msg;
    msg.data = 1.0;  // 夹爪打开状态，发布 1
    grasp_publisher_->publish(msg);
}
void ManipulatorAction::openGripper_max() {
    RCLCPP_INFO(node_->get_logger(), "Opening gripper.");
    gripper_group_.setNamedTarget("open_max");
    gripper_group_.move();
    // 发布夹爪打开的状态
    // std_msgs::msg::Float32 msg;
    // msg.data = 1.0;  // 夹爪打开状态，发布 1
    // grasp_publisher_->publish(msg);
}

// Attach 物体
void ManipulatorAction::attachObject(Object& object) {
    std::vector<std::string> touch_links;
    touch_links.push_back("link_right");
    touch_links.push_back("link_left");

    gripper_group_.attachObject(object.getCollisionObject().id, "link_hand", touch_links);
    RCLCPP_INFO(node_->get_logger(), "Object attached.");
}

// Detach 物体
    void ManipulatorAction::detachObject(Object& object) {
        gripper_group_.detachObject(object.getCollisionObject().id);
        RCLCPP_INFO(node_->get_logger(), "Object detached.");
    }

