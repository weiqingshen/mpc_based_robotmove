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

// ManipulatorAction 构造函数
ManipulatorAction::ManipulatorAction(const rclcpp::Node::SharedPtr& node,
                                     moveit::planning_interface::MoveGroupInterface& move_group,
                                     moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                     moveit::planning_interface::MoveGroupInterface& gripper_group)
        : node_(node), move_group_(move_group), planning_scene_interface_(planning_scene_interface), gripper_group_(gripper_group),
          trajectory_("default_trajectory.yaml") {  // 只传递 YAML 文件
    // 其他初始化代码...
    gripper_state_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("gripper_state", 10);
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
    moveToPose(grasp_offset_pose);

    // 2. 闭合夹爪
    closeGripper();

    // 3. Attach 物体
    attachObject(object);

    moveToPose(target_initial_pose);

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
        moveToPose(target_pose.getTargetPose());
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

    moveToPose(target_initial_pose);
    closeGripper();
    openGripper();
}
void ManipulatorAction::moveAction(Object& object, GraspPose& grasp_pose, const std::string& trajectory_name) {
    // 1. 获取抓取偏移姿态并移动到物体的位置
    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
if (!trajectory_name.empty()) {
    RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name.c_str());
    trajectory_.setYamlFileName(trajectory_name);
    trajectory_.executeTrajectory(move_group_, trajectory_name);
} else {
    RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
    // 4. 移动到目标位置
    // moveToPose(grasp_offset_pose);
}
moveToPose(grasp_offset_pose);
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

    // 1. 获取抓取偏移姿态并移动到物体的位置
    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
    if (!trajectory_name1.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name1.c_str());
        trajectory_.setYamlFileName(trajectory_name1);
        trajectory_.executeTrajectory(move_group_, trajectory_name1);
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        // 4. 移动到目标位置
        //moveToPose(target_pose.getTargetPose());
    }
    moveToPose(grasp_offset_pose);

    // 2. 闭合夹爪
    closeGripper();

    // 3. Attach 物体
    attachObject(object);
    // 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name2.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name2.c_str());
        trajectory_.setYamlFileName(trajectory_name2);
        trajectory_.executeTrajectory(move_group_, trajectory_name2);
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        // 4. 移动到目标位置
        //moveToPose(target_pose.getTargetPose());
    }

    geometry_msgs::msg::Pose target_initial_pose;
    target_initial_pose.position.x = 0.0363116;  // 替换为目标x坐标
    target_initial_pose.position.y = 0.204015;  // 替换为目标y坐标
    target_initial_pose.position.z = 0.401;  // 替换为目标z坐标
    target_initial_pose.orientation.x = 0.0;  // 替换为四元数x
    target_initial_pose.orientation.y = 1.0;  // 替换为四元数y
    target_initial_pose.orientation.z = 0.0;  // 替换为四元数z
    target_initial_pose.orientation.w = 0.0;  // 替换为四元数w

    moveToPose(target_initial_pose);

    // 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name3.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name3.c_str());
        trajectory_.setYamlFileName(trajectory_name3);
        trajectory_.executeTrajectory(move_group_, trajectory_name3);
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        // 4. 移动到目标位置
        //moveToPose(target_pose.getTargetPose());
    }
    moveToPose(target_pose.getTargetPose());//这一行必不可少 如果缺少了 可能无法完成物体的坐标更新
    // 5. 打开夹爪
    openGripper();

    // 6. Detach 物体
    detachObject(object);

    // 如果传递了轨迹的名字，先执行传递的轨迹
    if (!trajectory_name4.empty()) {
        RCLCPP_INFO(node_->get_logger(), "执行轨迹: %s", trajectory_name4.c_str());
        trajectory_.setYamlFileName(trajectory_name4);
        trajectory_.executeTrajectory(move_group_, trajectory_name4);
    } else {
        RCLCPP_INFO(node_->get_logger(), "没有提供轨迹名称，跳过轨迹执行步骤");
        // 4. 移动到目标位置
        //moveToPose(target_pose.getTargetPose());
    }
    // 7. 使用 TargetPose 的目标位置来更新物体的坐标
    geometry_msgs::msg::Pose final_pose = grasp_pose.getObjectOffsetPose(target_pose.getTargetPose());

    // 更新物体的坐标
    object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "物体坐标已更新到 (%f, %f, %f)", final_pose.position.x, final_pose.position.y, final_pose.position.z);

    moveToPose(target_initial_pose);
}




// 检查文件是否存在的函数
bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void ManipulatorAction::moveToPose(const geometry_msgs::msg::Pose& pose) {
    bool success = false;  // 记录是否成功
    int attempt_count = 0;  // 尝试次数
    const int max_attempts = 100;  // 最大尝试次数

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

    // 尝试最多 max_attempts 次规划和执行
    while (attempt_count < max_attempts && !success) {
        // 创建一个计划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // 尝试规划
        bool plan_success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_success) {
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "规划成功，尝试执行动作");

            // 按比例缩放轨迹时间，例如缩短为原来的2倍速度
            double scale_factor = 5; // 设置缩放比例
            scale_trajectory_time(my_plan.trajectory_, scale_factor);

            // 尝试执行计划
            moveit::core::MoveItErrorCode execute_result = move_group_.execute(my_plan);

            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "计划执行成功");
                success = true;  // 规划和执行都成功
            } else {
                // 执行失败，增加尝试次数
                attempt_count++;
                RCLCPP_WARN(rclcpp::get_logger("move_group_interface"), "计划执行失败，尝试第%d次", attempt_count);
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
    // std_msgs::msg::Float32 msg;
    // msg.data = 0.0;  // 夹爪闭合状态，发布 0
    // gripper_state_publisher_->publish(msg);
}

// 打开夹爪
void ManipulatorAction::openGripper() {
    RCLCPP_INFO(node_->get_logger(), "Opening gripper.");
    gripper_group_.setNamedTarget("open");
    gripper_group_.move();
    // 发布夹爪打开的状态
    // std_msgs::msg::Float32 msg;
    // msg.data = 1.0;  // 夹爪打开状态，发布 1
    // gripper_state_publisher_->publish(msg);
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

