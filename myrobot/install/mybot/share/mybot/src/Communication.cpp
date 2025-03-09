



#include "Communication.h"
#include <chrono>
#include <thread>
//后续添加可删除
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>

// 构造函数，初始化订阅者和发布者
Communication::Communication() : Node("communication_node"), received_valid_pose(false) {
    // 初始化订阅者，接收三个坐标
    pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "arm_pose", 10,
            std::bind(&Communication::poseArrayCallback, this, std::placeholders::_1)
    );

    // 初始化发布者，发送完成信号
    status_publisher = this->create_publisher<std_msgs::msg::Bool>("move_status", 10);

    // 初始化反馈发布者，发送消息接收成功信号
    feedback_publisher = this->create_publisher<std_msgs::msg::Bool>("feedback_topic", 10);
}

// 获取 position 结构体的引用，通过引用传递
void Communication::getPosition(position& target) const {
    target = target_position_;  // 通过引用传递，将 target_position_ 赋值给传入的 target
}

// 检查Pose是否有效的辅助函数
bool Communication::is_valid_pose(const geometry_msgs::msg::Pose& msg) {
    // 检查 x, y, z 是否为有效的浮点值
    if (std::isnan(msg.position.x) || std::isnan(msg.position.y) || std::isnan(msg.position.z)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid pose: NaN value detected in x, y, or z.");
        return false;
    }
    return true;
}

// 回调函数，用于接收并更新目标位置
void Communication::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (received_valid_pose) {
        // 指定要保存文本的文件路径和文件名
        std::string filePath = "/home/fins/dev_ws/time_check.txt"; // Linux 示例
        // 要保存的文本
        std::string textToSave = "上位机收到消息";
        // 获取当前时间戳
        auto now = std::chrono::system_clock::now();
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&now_time_t);
        // 格式化时间戳为字符串
        std::ostringstream timestampStream;
        timestampStream << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");
        std::string currentTimestamp = timestampStream.str();
        // 打开文件并写入内容，如果文件不存在则会自动创建
        std::ofstream outputFile(filePath, std::ios::app);
        if (outputFile.is_open()) {
            outputFile << textToSave << std::endl;
            outputFile << "当前时间戳: " << currentTimestamp << std::endl;
            outputFile.close();
            std::cout << "文本和时间戳已保存到 " << filePath << std::endl;
        } else {
            std::cerr << "无法打开文件 " << filePath << std::endl;
        }
        return; // 已经接收到有效消息，忽略后续消息
    }

    // 确保消息中包含三个位置
    if (msg->poses.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Received PoseArray contains less than 3 positions.");
        return;
    }

    // 检查每个pose是否有效
    for (size_t i = 0; i < 3; ++i) {
        if (!is_valid_pose(msg->poses[i])) {
            RCLCPP_ERROR(this->get_logger(), "Invalid pose detected at index %ld.", i);
            return; // 如果任何一个pose无效，忽略这个消息
        }
    }

    // 更新结构体内的坐标
    target_position_.x1 = msg->poses[0].position.x;
    target_position_.y1 = msg->poses[0].position.y;
    target_position_.order1=msg->poses[0].position.z;
    target_position_.x2 = msg->poses[1].position.x;
    target_position_.y2 = msg->poses[1].position.y;
    target_position_.order2=msg->poses[1].position.z;
    target_position_.x3 = msg->poses[2].position.x;
    target_position_.y3 = msg->poses[2].position.y;
    target_position_.order3 = msg->poses[2].position.z;


    RCLCPP_INFO(this->get_logger(), "Received target positions: x1=%f, y1=%f, x2=%f, y2=%f, x3=%f, y3=%f",
                target_position_.x1, target_position_.y1, target_position_.x2, target_position_.y2,
                target_position_.x3, target_position_.y3);

    // 标记为接收到有效位置信息
    if(is_processing_message)
    received_valid_pose = true;

    // 发布反馈消息，通知发送端已成功接收
    auto feedback_msg = std_msgs::msg::Bool();
    feedback_msg.data = true; // true 表示接收成功
    feedback_publisher->publish(feedback_msg);

    RCLCPP_INFO(this->get_logger(), "Feedback sent: PoseArray received successfully.");
}

void Communication::waitForMessage() {
    is_processing_message=true;
    // 每两秒检查一次是否接收到成功的消息
    while (rclcpp::ok()) {
        //RCLCPP_INFO(this->get_logger(), "Checking received_valid_pose: %s", received_valid_pose ? "true" : "false");
        if (received_valid_pose) {
            RCLCPP_INFO(this->get_logger(), "Message received successfully. Exiting loop.");
            received_valid_pose = false;  // 重置标志以接受下一个消息
            is_processing_message=false;
            break;  // 退出循环
        } /*else {
            RCLCPP_INFO(this->get_logger(), "Waiting for message...");
            //std::this_thread::sleep_for(std::chrono::seconds(2));  // 每两秒等待
        }*/
    }
}


// 发送运动完成状态
void Communication::sendMoveStatus(bool status) {
    auto msg = std_msgs::msg::Bool();
    msg.data = status;
    status_publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Movement completed. Status sent: %s", status ? "true" : "false");
}
