#ifndef SIM_ROBOT_HARDWARE_DIFFBOT_HPP_
#define SIM_ROBOT_HARDWARE_DIFFBOT_HPP_

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace hardware {

// 差速模型硬件设备
// 模拟电机，执行驱动电机和读取电机选装角度信息
// 必须被定时驱动，否则上一个控制指令不会停止
class DeviceDiffBot {
   public:
    DeviceDiffBot(rclcpp::Node::SharedPtr);

    DeviceDiffBot(const DeviceDiffBot&) = default;
    DeviceDiffBot(DeviceDiffBot&&) = default;
    DeviceDiffBot& operator=(const DeviceDiffBot&) = delete;
    DeviceDiffBot& operator=(DeviceDiffBot&&) = delete;

    bool write(const std::vector<double>& commamd);
    const std::vector<double>& read() const;

   private:
    bool publish();
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> publisher_;

    std::chrono::time_point<std::chrono::system_clock> last_time_point_;
    const std::vector<std::string> name_;
    std::vector<double> position_{0, 0};        // 上次轮子角度
    std::vector<double> velocity_{0, 0};        // 上次轮子角速度
    std::vector<double> position_count_{0, 0};  // 统计轮子转的总角度
};

// 差速模型硬件系统
// 读取目标速度，并实际下发控制动作
class SystemDiffBot {
   public:
    SystemDiffBot(rclcpp::Node::SharedPtr);

   private:
    std::shared_ptr<DeviceDiffBot> device_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped> sub_vel_;
    geometry_msgs::msg::TwistStamped::SharedPtr last_vel_;

    std::mutex mutex_;
};

}  // namespace hardware

#endif  // SIM_ROBOT_HARDWARE_DIFFBOT_HPP_