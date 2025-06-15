#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

class WheelVelocities : public rclcpp::Node {
public:
  WheelVelocities() : Node("wheel_velocities") {
    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node.");
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);
  }

  void publish_wheel_velocities() {
    while (pub_->get_subscription_count() == 0) {
      rclcpp::sleep_for(100ms);
    }
    move_forward();
    rclcpp::sleep_for(3s);
    move_backward();
    rclcpp::sleep_for(3s);
    move_right();
    rclcpp::sleep_for(3s);
    move_left();
    rclcpp::sleep_for(3s);
    move_clockwise();
    rclcpp::sleep_for(3s);
    move_counterclockwise();
    rclcpp::sleep_for(3s);
    stop();

    return;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  std_msgs::msg::Float32MultiArray msg;

  void move_forward() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {10.0, 10.0, 10.0, 10.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving forward");
  }

  void move_backward() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {-10.0, -10.0, -10.0, -10.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving backward");
  }

  void move_right() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {10.0, -10.0, 10.0, -10.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving right");
  }

  void move_left() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {-10.0, 10.0, -10.0, 10.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving left");
  }

  void move_clockwise() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {10.0, -10.0, -10.0, 10.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving clockwise");
  }

  void move_counterclockwise() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {-10.0, 10.0, 10.0, -10.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving counterclockwise");
  }

  void stop() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {0.0, 0.0, 0.0, 0.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Stop");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelocities>();
  node->publish_wheel_velocities();
  rclcpp::shutdown();
  return 0;
}