#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// 设置物体位置结构体
struct position {
    float x1, y1, x2, y2, x3, y3;
    int order1,order2,order3;
};

class Communication : public rclcpp::Node {
public:
    Communication();  // 构造函数
    void getPosition(position& target) const;  // 修改为引用传递
    void sendMoveStatus(bool status);  // 发布运动完成状态
    void waitForMessage();  // 阻塞方法，等待接收消息

    bool received_valid_pose;  // 标记是否接收到有效的位置信息

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);  // 订阅回调函数
    bool is_valid_pose(const geometry_msgs::msg::Pose& msg);  // 检查Pose是否有效的辅助函数
    bool is_processing_message=true;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_subscriber;  // 订阅者
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher;  // 发布者，发送运动状态
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feedback_publisher;  // 发布者，发送消息接收成功反馈

    position target_position_;  // 存储接收到的目标位置
};

#endif // COMMUNICATION_H
