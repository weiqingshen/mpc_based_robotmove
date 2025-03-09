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


bool is_gripper_operating = false;  // 夹爪是否正在操作

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

    if (is_first_recv_message) {
        // 初始化记录，仅记录前 6 个关节
        recv_last_joint_positions = std::vector<double>(msg->position.begin(), msg->position.end() - 1);
        recv_last_gripper_value = msg->position.back();  // 记录夹爪值
        is_first_recv_message = false;
        joint_state_queue_.push(*msg);  // 首次消息直接进入队列
        return;
    }

    // 检查是否有任何数据发生变化
    std::vector<double> current_joint_positions(msg->position.begin(), msg->position.end() - 1);
    double current_gripper_value = msg->position.back();

    if (!are_positions_equal(recv_last_joint_positions, current_joint_positions) ||
        std::fabs(current_gripper_value - recv_last_gripper_value) > 1e-5) {
        joint_state_queue_.push(*msg);  // 只要有变化就入队
    }

    // 更新接收状态的记录
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

    if (!joint_state_queue_.empty()) {
        auto msg = joint_state_queue_.front();
        joint_state_queue_.pop();

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
            //if(gripper_msg.data==0.0)
            	//sleep(0.1);
            gripper_publisher->publish(gripper_msg);
            gripper_publisher->publish(gripper_msg);
            gripper_publisher->publish(gripper_msg);
            gripper_publisher->publish(gripper_msg);
            gripper_publisher->publish(gripper_msg);
            //if(gripper_msg.data==1.0)
                //sleep(0.5);
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


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("trajectory_buffer_node");

    // 打开第一个 YAML 文件
    open_new_yaml_file();

    // 订阅 JointState 话题
    auto joint_state_subscription = node->create_subscription<JointStateMsg>(
        "/joint_states", 500, joint_state_callback);

    // 创建发布器，用于发布 JointTrajectoryPoint 消息
    auto joint_publisher = node->create_publisher<JointTrajectoryPointMsg>("/buffered_display_trajectory_point", 500);

    // 创建发布器，用于发布夹爪状态
    auto gripper_publisher = node->create_publisher<std_msgs::msg::Float32>("/buffered_gripper_state", 500);

    // 定时器：每 10 毫秒检查队列并发布数据
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(17),
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
