#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>               // JointState 消息
#include <trajectory_msgs/msg/joint_trajectory_point.hpp> // JointTrajectoryPoint 消息
#include <std_msgs/msg/float32.hpp>                      // Float32 消息
#include <queue>
#include <mutex>
#include <vector>
#include <cmath>
#include <fstream>   // 用于保存 YAML 文件
#include <iomanip>   // 控制输出格式
#include <chrono>
#include <thread>
#include <sstream>   // 用于字符串流
#include "std_msgs/msg/string.hpp"
#include <yaml-cpp/yaml.h>  // 使用 YAML 库
#include <moveit_msgs/msg/robot_trajectory.hpp>
bool is_gripper_operating = false;  // 夹爪是否正在操作
bool is_executing_trajectory = false;  // 是否正在执行轨迹

using JointStateMsg = sensor_msgs::msg::JointState;
using JointTrajectoryPointMsg = trajectory_msgs::msg::JointTrajectoryPoint;

std::queue<JointStateMsg> joint_state_queue_;
std::mutex queue_mutex_;  // 确保队列线程安全
int num=0;
// 接收状态记录
std::vector<double> recv_last_joint_positions;  // 接收状态：保存上一次的关节位置
double recv_last_gripper_value = 0.0;           // 接收状态：保存夹爪的上次值
bool is_first_recv_message = true;              // 是否是首次接收消息

// 发布状态记录
std::vector<double> pub_last_joint_positions;  // 发布状态：保存上一次的关节位置
double pub_last_gripper_value = 0.0;           // 发布状态：保存夹爪的上次值
bool is_first_pub_message = true;              // 是否是首次发布消息

bool is_save=false;


// YAML 文件相关变量
std::ofstream yaml_file;
int yaml_file_counter = 1;  // 计数 YAML 文件编号
uint64_t time_from_start_ns = 0;  // 模拟时间偏移（单位：纳秒）

// 读取 YAML 文件中的目标关节位置
std::vector<std::vector<double>> yaml_trajectory_positions;  // 存储 YAML 中的轨迹点
void read_trajectory_from_yaml(const std::string& filename) {
    // 尝试打开文件并检查是否成功
    RCLCPP_INFO(rclcpp::get_logger("YAML"), "尝试打开文件: %s", filename.c_str());

    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("YAML"), "无法打开文件: %s", filename.c_str());
        return;
    }

    YAML::Node config = YAML::LoadFile(filename);  // 加载 YAML 文件
    yaml_trajectory_positions.clear();  // 清空旧的轨迹数据

    // 遍历并处理每个轨迹点
    for (const auto& trajectory : config["trajectories"]) {
        for (const auto& point : trajectory["points"]) {
            std::vector<double> joint_positions = point["joint_positions"].as<std::vector<double>>();

            // 创建 JointStateMsg 消息
            JointStateMsg joint_state_msg;
            joint_state_msg.header.stamp = rclcpp::Clock().now();
            joint_state_msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            joint_state_msg.position = joint_positions;

            // 入队操作
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                joint_state_queue_.push(joint_state_msg);
            }

            // 输出关节位置
            std::ostringstream joint_positions_str;
            for (const auto& pos : joint_state_msg.position) {
                joint_positions_str << pos << " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("Trajectory"), "成功入队: [%s]", joint_positions_str.str().c_str());
        }
    }

    file.close();
}




// 打开新的 YAML 文件
void open_new_yaml_file() {
    if (yaml_file.is_open()) {
        yaml_file.close();  // 如果之前有打开的文件，先关闭
    }

    // 构建文件名：trajectory_<number>.yaml
    std::string filename = "trajectory_" + std::to_string(yaml_file_counter++) + ".yaml";

    // 打开新的文件
    yaml_file.open(filename);

    // 写入 YAML 文件顶层结构 "trajectories"
    yaml_file << "trajectories:\n";  // 这是你需要的顶层字段
    yaml_file << "  - name: \"trajectory_" << yaml_file_counter - 1 << "\"\n";  // 当前文件的 trajectory 名称
    yaml_file << "    points:\n";  // 准备写入点

    time_from_start_ns=0;

    // 打印日志，确认新文件已打开
    RCLCPP_INFO(rclcpp::get_logger("YAML"), "Opened new YAML file: %s", filename.c_str());
}


// 将机械臂数据保存到 YAML 文件
void save_to_yaml(const std::vector<double>& positions, uint64_t time_from_start_ns) {
    // 如果文件未打开，初始化 YAML 文件
    if (!yaml_file.is_open()) {
        open_new_yaml_file();
    }

    // 写入当前点的信息
    yaml_file << "      - name: \"point_" << num << "\"\n";
    yaml_file << "        time_from_start: [0, " << time_from_start_ns << "]\n";
    yaml_file << "        joint_positions: [";

    // 写入关节位置数据
    for (size_t i = 0; i < positions.size(); ++i) {
        yaml_file << std::fixed << std::setprecision(3) << positions[i];
        if (i != positions.size() - 1) {
            yaml_file << ", ";
        }
    }
    yaml_file << "]\n";
}



// 检查两个关节数组是否完全相等
bool are_positions_equal(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {  // 严格比较是否相等
            return false;
        }
    }
    return true;
}

void publish_arm_and_gripper(std::shared_ptr<rclcpp::Publisher<JointTrajectoryPointMsg>> joint_publisher,
                             std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gripper_publisher) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 如果夹爪正在操作，暂停队列的读取
    if (is_gripper_operating) {
        return;  // 跳过本次读取
    }

    static size_t point_counter = 0;  // 用于计数中间点是否达到5的倍数

    // 如果队列不为空
    if (!joint_state_queue_.empty()) {
        auto msg = joint_state_queue_.front();
        joint_state_queue_.pop();

        // // 打印出队的数据数量和开头部分
        // std::ostringstream oss;
        // oss << "Dequeued joint states (Total " << msg.position.size() << " values): [";
        //
        // // 遍历打印每个关节位置数据
        // for (size_t i = 0; i < msg.position.size(); ++i) {
        //     oss << std::fixed << std::setprecision(3) << msg.position[i];  // 打印数字保留3位小数
        //     if (i < msg.position.size() - 1) {
        //         oss << ", ";  // 在每个值后添加逗号分隔符
        //     }
        // }
        //
        // // 打印结尾部分
        // oss << "]";
        //
        // // 打印完整信息
        // RCLCPP_INFO(rclcpp::get_logger("JointStateQueue"), "%s", oss.str().c_str());

        // 判断数据数量
        if (msg.position.size() == 6) {
            // 如果队列中有 6 个数据，直接发布机械臂状态
            JointTrajectoryPointMsg trajectory_msg;

            // 直接使用关节数据，保持原顺序
            trajectory_msg.positions = {msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5]};
            joint_publisher->publish(trajectory_msg);

            num++;

            // 更新机械臂状态记录
            pub_last_joint_positions = msg.position;

            if (is_save) {
                // 更新时间偏移并保存到 YAML
                time_from_start_ns += 10000000;  // 假设每个点间隔 10 毫秒
                save_to_yaml(trajectory_msg.positions, time_from_start_ns);
            }

            return;  // 返回，不处理夹爪数据
        } else if (msg.position.size() == 1) {
            // 如果队列中只有 1 个数据，说明是夹爪控制数据
            double current_gripper_value = msg.position.front();  // 获取夹爪状态值

            // 在发指令前标记夹爪操作进行中
            is_gripper_operating = true;

            // 发布夹爪指令前打开新的 YAML 文件
            open_new_yaml_file();

            std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

            // 发送夹爪状态信息
            std_msgs::msg::Float32 gripper_msg;
            gripper_msg.data = (current_gripper_value == 1.0) ? 1.0 : 0.0;  // 判断是打开还是关闭
            RCLCPP_INFO(rclcpp::get_logger("GripperState"), "Gripper is %s (Value: %.3f)",
                        (gripper_msg.data == 1.0 ? "opening" : "closing"), current_gripper_value);

            gripper_publisher->publish(gripper_msg);  // 发布夹爪状态消息
            gripper_publisher->publish(gripper_msg);  // 重复发送几次确保收到消息
            gripper_publisher->publish(gripper_msg);
            gripper_publisher->publish(gripper_msg);
            gripper_publisher->publish(gripper_msg);

            std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

            // 更新夹爪状态记录
            pub_last_gripper_value = current_gripper_value;

            // 在夹爪操作完成后恢复标志
            is_gripper_operating = false;
        }

        // 计数器增加
        point_counter++;
    }
}


// 新的回调函数：处理 gripper_control 消息
void gripper_control_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 创建 JointStateMsg 消息并入队
    JointStateMsg joint_state_msg;
    joint_state_msg.header.stamp = rclcpp::Clock().now();  // 设置时间戳
    joint_state_msg.name = {"gripper"};  // 夹爪只有一个控制信号
    joint_state_msg.position = {msg->data};  // 夹爪的状态（打开或关闭）

    // 将夹爪控制状态数据推入队列
    joint_state_queue_.push(joint_state_msg);

    // 打印入队的夹爪状态数据
    RCLCPP_INFO(rclcpp::get_logger("GripperControl"), "Gripper control data queued: %.3f", msg->data);
}



void trajectory_status_callback(const std_msgs::msg::String::SharedPtr msg) {
    // 判断消息中是否包含"开始执行"或"执行完成"
    if (msg->data.find("开始执行") != std::string::npos) {
        // 当收到 "轨迹名字+开始执行" 消息时
        is_executing_trajectory = true;
        RCLCPP_INFO(rclcpp::get_logger("TrajectoryControl"), "开始执行轨迹: %s", msg->data.c_str());

        // 去掉 "开始执行" 字符串
        std::string trajectory_name = msg->data;
        size_t pos = trajectory_name.find("开始执行");  // 查找 "开始执行" 字符串的位置
        if (pos != std::string::npos) {
            trajectory_name = trajectory_name.substr(0, pos);  // 提取 "开始执行" 前的部分
        }

        // 去除末尾空格
        trajectory_name = trajectory_name.erase(trajectory_name.find_last_not_of(" \t\r\n") + 1);
        RCLCPP_INFO(rclcpp::get_logger("TrajectoryControl"), "执行轨迹: %s", trajectory_name.c_str());

        // 构造文件路径并读取 YAML 文件
        std::string file_path = "/home/fins/myrobot_final/src/mybot/config/" + trajectory_name + ".yaml";

        read_trajectory_from_yaml(file_path);  // 读取目标轨迹文件

    }
    else if (msg->data.find("执行完成") != std::string::npos) {
        // 当收到 "执行完成" 消息时
        is_executing_trajectory = false;
        RCLCPP_INFO(rclcpp::get_logger("TrajectoryControl"), "轨迹执行完成: %s", msg->data.c_str());
    } else {
        RCLCPP_WARN(rclcpp::get_logger("TrajectoryControl"), "收到未知的轨迹状态消息: %s", msg->data.c_str());
    }
}


void manipulator_trajectory_callback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (msg->joint_trajectory.points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("ManipulatorTrajectory"), "Received empty trajectory points.");
        return;
    }

    // 遍历每个轨迹点
    for (const auto& point : msg->joint_trajectory.points) {
        std::vector<double> joint_positions = point.positions;

        if (joint_positions.size() == 6) {
            // 重新调整关节顺序为 2, 3, 1, 4, 5, 6
            // std::vector<double> reordered_positions = {
            //     joint_positions[1],  // joint_2
            //     joint_positions[2],  // joint_3
            //     joint_positions[0],  // joint_1
            //     joint_positions[3],  // joint_4
            //     joint_positions[4],  // joint_5
            //     joint_positions[5]   // joint_6
            // };

            // 创建 JointStateMsg 消息并填充数据
            JointStateMsg joint_state_msg;
            joint_state_msg.header.stamp = rclcpp::Clock().now();  // 设置时间戳
            joint_state_msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            joint_state_msg.position = joint_positions;  // 设置重新排序的关节位置

            // 将关节状态数据推入队列
            joint_state_queue_.push(joint_state_msg);

            // 简化打印关节值
            std::ostringstream joint_positions_str;
            joint_positions_str << "[";
            for (size_t i = 0; i < joint_positions.size(); ++i) {
                joint_positions_str << joint_positions[i];
                if (i < joint_positions.size() - 1) {
                    joint_positions_str << ", ";
                }
            }
            joint_positions_str << "]";

            RCLCPP_INFO(rclcpp::get_logger("ManipulatorTrajectory"), joint_positions_str.str().c_str());
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ManipulatorTrajectory"), "Invalid number of joint positions received.");
        }
    }
}



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("trajectory_buffer_node");

    // 打开第一个 YAML 文件
    open_new_yaml_file();

    // 修改为 "trajectory_status" 话题
    auto trajectory_start_subscription = node->create_subscription<std_msgs::msg::String>(
    "/trajectory_status", 100, trajectory_status_callback);


    // 订阅 gripper_control 话题（新的订阅）
    auto gripper_control_subscription = node->create_subscription<std_msgs::msg::Float32>(
        "gripper_control", 100, gripper_control_callback);

    // 订阅 manipulator_trajectory 话题（新的订阅）
    auto manipulator_trajectory_subscription = node->create_subscription<moveit_msgs::msg::RobotTrajectory>(
        "manipulator_trajectory", 100, manipulator_trajectory_callback);

    // 创建发布器，用于发布 JointTrajectoryPoint 消息
    auto joint_publisher = node->create_publisher<JointTrajectoryPointMsg>("/buffered_display_trajectory_point", 500);

    // 创建发布器，用于发布夹爪状态
    auto gripper_publisher = node->create_publisher<std_msgs::msg::Float32>("/buffered_gripper_state", 500);

    // 定时器：每 10 毫秒检查队列并发布数据
    auto timer = node->create_wall_timer(

        std::chrono::milliseconds(50),
        [joint_publisher, gripper_publisher]() {
            publish_arm_and_gripper(joint_publisher, gripper_publisher);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();

    // 关闭 YAML 文件
    if (yaml_file.is_open()) {
        yaml_file.close();
    }
    return 0;
}
