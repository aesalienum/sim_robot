
#include "hardware/diffbot.h"

#include <cassert>
#include <cmath>

// #include <cstddef>

namespace {
const double PI = std::acos(-1);

double limit_circle(const double in) {
    double out = in;
    while (true) {
        if (out < 0) {
            out += 2 * PI;
        } else if (out >= 2 * PI) {
            out -= 2 * PI;
        } else {
            return out;
        }
    }
}
}  // namespace

namespace hardware {
DeviceDiffBot::DeviceDiffBot(rclcpp::Node::SharedPtr node)
    : node_(node),
      publisher_(node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1)),
      last_time_point_(std::chrono::system_clock::now()),
      name_({node_->get_parameter("joint_name_left_wheel").as_string(),
             node_->get_parameter("joint_name_left_wheel").as_string()}){};

bool DeviceDiffBot::write(const std::vector<double>& commamd) {
    assert(commamd.size() == position_count_.size());

    auto now_time_point = std::chrono::system_clock::now();
    auto use_time = std::chrono::duration_cast<std::chrono::microseconds>(now_time_point - last_time_point_);
    for (size_t i = 0; i < position_count_.size(); ++i) {
        auto sub = use_time.count() * 1e-6 * velocity_[i];
        // TODO 约束加速度
        position_count_[i] += sub;
        position_[i] = limit_circle(position_[i] + sub);
        velocity_[i] = commamd[i];
    }
    return publish();
};

const std::vector<double>& DeviceDiffBot::read() const {
    return position_count_;
};

bool DeviceDiffBot::publish() {
    auto msg = sensor_msgs::msg::JointState();

    msg.header.stamp = node_->now();
    msg.name = name_;
    msg.position = position_;
    msg.velocity = velocity_;
    publisher_->publish(msg);
    return true;
};
}  // namespace hardware
