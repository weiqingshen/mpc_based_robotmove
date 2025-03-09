#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include <serial/serial.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <queue>
using namespace std;

class TrajectorySubscriber : public rclcpp::Node
{
public:
    TrajectorySubscriber() : Node("trajectory_subscriber")
    {
        // 串口设置
        string port = "/dev/ttyUSB0";
        uint32_t baud_rate = 921600;
        serial_.setPort(port);
        serial_.setBaudrate(baud_rate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);

        try {
            serial_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口");
        }

        if (serial_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "串口已初始化");
        } else {
            RCLCPP_ERROR(this->get_logger(), "串口未能初始化");
        }

        // 订阅 /display_planned_path 话题
        subscription_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10,
            std::bind(&TrajectorySubscriber::topic_callback, this, std::placeholders::_1));

        // 创建定时器，每 100 毫秒触发一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectorySubscriber::send_next_data, this));
    }

    ~TrajectorySubscriber() {
        serial_.close();
    }

private:
    const uint16_t FRAME_HEADER = 0xAABB;
    const uint16_t FRAME_TRAILER = 0xCCDD;

    void topic_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
    {
        // 清空队列
        data_queue_ = std::queue<std::vector<uint8_t>>();

        // 遍历所有轨迹
        for (const auto& robot_trajectory : msg->trajectory) {
            const auto& joint_trajectory = robot_trajectory.joint_trajectory;

            // 遍历每个轨迹点
            for (const auto& point : joint_trajectory.points) {
                std::vector<uint8_t> buffer;

                // 添加帧头
                buffer.push_back(static_cast<uint8_t>((FRAME_HEADER >> 8) & 0xFF));
                buffer.push_back(static_cast<uint8_t>(FRAME_HEADER & 0xFF));

                // 逐个将浮点数添加到缓冲区
                for (size_t i = 0; i < point.positions.size(); ++i) {
                    float pos = point.positions[i];

                    if(i==1)
                        pos=-pos;

                    // 打印处理后的角度值
                    RCLCPP_INFO(this->get_logger(), "Joint %ld adjusted position: %f", i + 1, pos);


                    // 将浮点数转换为 uint32_t，然后拆分为字节添加到缓冲区
                    uint32_t as_int = *reinterpret_cast<uint32_t*>(&pos);
                    buffer.push_back(static_cast<uint8_t>(as_int & 0xFF));         // 低字节
                    buffer.push_back(static_cast<uint8_t>((as_int >> 8) & 0xFF));
                    buffer.push_back(static_cast<uint8_t>((as_int >> 16) & 0xFF));
                    buffer.push_back(static_cast<uint8_t>((as_int >> 24) & 0xFF)); // 高字节
                }


                // 添加帧尾
                buffer.push_back(static_cast<uint8_t>((FRAME_TRAILER >> 8) & 0xFF));
                buffer.push_back(static_cast<uint8_t>(FRAME_TRAILER & 0xFF));

                // 将完整的数据包放入队列
                data_queue_.push(buffer);
            }
        }
    }

    void send_next_data()
    {
        if (!serial_.isOpen() || data_queue_.empty()) return;

        // 从队列中取出下一组数据并发送
        const std::vector<uint8_t>& buffer = data_queue_.front();
        serial_.write(buffer.data(), buffer.size());  // 发送缓冲区数据
        std::cout << "Sent: " << std::endl;
        for (auto byte : buffer) {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " "; }
        std::cout << std::dec << std::endl; // 切换回十进制输出
        data_queue_.pop(); // 发送后将其从队列中移除
    }

    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::queue<std::vector<uint8_t>> data_queue_;  // 用于存储待发送的数据
    mutable serial::Serial serial_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectorySubscriber>());
    rclcpp::shutdown();
    return 0;
}
