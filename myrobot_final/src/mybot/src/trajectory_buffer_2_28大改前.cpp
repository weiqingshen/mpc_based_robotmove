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
    // 输出尝试打开的路径
    RCLCPP_INFO(rclcpp::get_logger("YAML"), "尝试打开 YAML 文件: %s", filename.c_str());

    // 读取指定文件路径的 YAML
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("YAML"), "无法打开文件: %s", filename.c_str());
        return;
    }

    YAML::Node config = YAML::LoadFile(filename);  // 加载 YAML 文件
    yaml_trajectory_positions.clear();  // 清空旧的轨迹数据

    // 解析 YAML 文件中的轨迹数据
    for (const auto& trajectory : config["trajectories"]) {
        for (const auto& point : trajectory["points"]) {
            std::vector<double> joint_positions = point["joint_positions"].as<std::vector<double>>();

            // 创建 JointStateMsg 消息
            JointStateMsg joint_state_msg;
            joint_state_msg.header.stamp = rclcpp::Clock().now();  // 设置时间戳
            joint_state_msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};  // 假设有6个关节
            joint_state_msg.position = joint_positions;  // 设置关节位置

            // 将 JointStateMsg 推入队列
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);  // 确保线程安全
                joint_state_queue_.push(joint_state_msg);  // 推入队列
            }

            // 打印入队的关节位置数据（所有位置在同一行）
            std::stringstream joint_positions_str;
            for (size_t i = 0; i < joint_state_msg.position.size(); ++i) {
                joint_positions_str << "关节 " << i + 1 << ": " << joint_state_msg.position[i] << " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("Trajectory"), "成功入队: %s", joint_positions_str.str().c_str());
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

// 接收状态：将 JointState 数据存入队列
void joint_state_callback(const JointStateMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 如果正在执行轨迹，跳过 joint_state 数据的接收
    if (is_executing_trajectory) {
         //如果处于轨迹执行中，我们从 YAML 文件中读取当前位置并入队
         // if (!yaml_trajectory_positions.empty()) {
         //     static size_t yaml_index = 0;  // 用于跟踪 YAML 中的位置
         //     if (yaml_index < yaml_trajectory_positions.size()) {
         //         JointStateMsg trajectory_msg;
         //         trajectory_msg.position = yaml_trajectory_positions[yaml_index];
         //         joint_state_queue_.push(trajectory_msg);  // 将读取的目标位置入队
         //         yaml_index++;
         //     }
         // }
        return;  // 不再处理 JointState 消息
    }

    if (is_first_recv_message) {
        recv_last_joint_positions = std::vector<double>(msg->position.begin(), msg->position.end() - 1);
        recv_last_gripper_value = msg->position.back();  // 记录夹爪值
        is_first_recv_message = false;
        joint_state_queue_.push(*msg);  // 首次消息直接进入队列
        return;
    }

    std::vector<double> current_joint_positions(msg->position.begin(), msg->position.end() - 1);
    double current_gripper_value = msg->position.back();

    if (!are_positions_equal(recv_last_joint_positions, current_joint_positions) ||
        std::fabs(current_gripper_value - recv_last_gripper_value) > 1e-5) {
        joint_state_queue_.push(*msg);  // 只要有变化就入队
        }

    recv_last_joint_positions = current_joint_positions;
    recv_last_gripper_value = current_gripper_value;
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
        // 打印队列中一共有多少个数据
        auto msg = joint_state_queue_.front();
        joint_state_queue_.pop();

        // 打印出队的数据数量和开头部分
        std::ostringstream oss;
        oss << "Dequeued joint states (Total " << msg.position.size() << " values): [";

        // 遍历打印每个关节位置数据
        for (size_t i = 0; i < msg.position.size(); ++i) {
            oss << std::fixed << std::setprecision(3) << msg.position[i];  // 打印数字保留3位小数
            if (i < msg.position.size() - 1) {
                oss << ", ";  // 在每个值后添加逗号分隔符
            }
        }

        // 打印结尾部分
        oss << "]";

        // 打印完整信息
        RCLCPP_INFO(rclcpp::get_logger("JointStateQueue"), "%s", oss.str().c_str());


        // 判断数据数量
        if (msg.position.size() == 6) {
            // 如果是6个点，直接发布机械臂状态并返回
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
        }

        // 当前消息的关节状态
        std::vector<double> current_joint_positions(msg.position.begin(), msg.position.end() - 1);
        double current_gripper_value = msg.position.back();

        if (is_first_pub_message) {
            // 首次发布仅发布机械臂状态
            JointTrajectoryPointMsg trajectory_msg;

            // 调整机械臂的关节顺序为 2, 3, 1, 4, 5, 6
            trajectory_msg.positions = {current_joint_positions[2], current_joint_positions[0],
                                         current_joint_positions[1], current_joint_positions[3],
                                         current_joint_positions[4], current_joint_positions[5]};
            joint_publisher->publish(trajectory_msg);

            // 初始化发布状态记录
            pub_last_joint_positions = current_joint_positions;
            pub_last_gripper_value = current_gripper_value;
            is_first_pub_message = false;
            point_counter = 1; // 计数器初始化为1
            time_from_start_ns = 0;  // 初始化时间偏移
            if(is_save) {
                save_to_yaml(trajectory_msg.positions, time_from_start_ns);  // 保存到 YAML
            }

            return;  // 不发布夹爪状态
        }

        // 发布机械臂状态
        if (!are_positions_equal(pub_last_joint_positions, current_joint_positions) || point_counter % 5 == 0) {
            JointTrajectoryPointMsg trajectory_msg;

            // 调整机械臂的关节顺序为 2, 3, 1, 4, 5, 6
            trajectory_msg.positions = {current_joint_positions[2], current_joint_positions[0],
                                         current_joint_positions[1], current_joint_positions[3],
                                         current_joint_positions[4], current_joint_positions[5]};
            joint_publisher->publish(trajectory_msg);

            num++;

            // 更新机械臂状态记录
            pub_last_joint_positions = current_joint_positions;

            if(is_save) {
                // 更新时间偏移并保存到 YAML
                time_from_start_ns += 10000000;  // 假设每个点间隔 10 毫秒
                save_to_yaml(trajectory_msg.positions, time_from_start_ns);
            }
        }

        // 发布夹爪状态
        if (current_gripper_value != pub_last_gripper_value) {
            // 在发指令前标记夹爪操作进行中
            is_gripper_operating = true;

            // 发布夹爪指令前打开新的 YAML 文件
            open_new_yaml_file();

            JointTrajectoryPointMsg trajectory_msg;

            // 调整机械臂的关节顺序为 2, 3, 1, 4, 5, 6
            trajectory_msg.positions = {current_joint_positions[2], current_joint_positions[0],
                                         current_joint_positions[1], current_joint_positions[3],
                                         current_joint_positions[4], current_joint_positions[5]};
            joint_publisher->publish(trajectory_msg);

            std_msgs::msg::Float32 gripper_msg;
            std::this_thread::sleep_for(std::chrono::duration<double>(0.3));
            num = 0;
            gripper_msg.data = (current_gripper_value < pub_last_gripper_value) ? 1.0 : 0.0;
            RCLCPP_INFO(rclcpp::get_logger("GripperState"), "Gripper is %s (Value: %.3f)",
                        (gripper_msg.data == 1.0 ? "opening" : "closing"), current_gripper_value);
            gripper_publisher->publish(gripper_msg);
            gripper_publisher->publish(gripper_msg);
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




int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("trajectory_buffer_node");

    // 打开第一个 YAML 文件
    open_new_yaml_file();

    // 修改为 "trajectory_status" 话题
    auto trajectory_start_subscription = node->create_subscription<std_msgs::msg::String>(
    "/trajectory_status", 100, trajectory_status_callback);

    // 订阅 JointState 话题
    auto joint_state_subscription = node->create_subscription<JointStateMsg>(
        "/joint_states", 500, joint_state_callback);

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
