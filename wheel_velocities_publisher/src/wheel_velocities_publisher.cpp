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
    this->publish_wheel_velocities();
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  std_msgs::msg::Float32MultiArray msg;

  void move_forward() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {1.0, 1.0, 1.0, 1.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving forward");
    rclcpp::sleep_for(3s);
  }

  void move_backward() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {-1.0, -1.0, -1.0, -1.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving backward");
    rclcpp::sleep_for(3s);
  }

  void move_right() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {1.0, -1.0, 1.0, -1.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving right");
    rclcpp::sleep_for(3s);
  }

  void move_left() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {1.0, -1.0, 1.0, -1.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving left");
    rclcpp::sleep_for(3s);
  }

  void move_clockwise() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {1.0, -1.0, -1.0, 1.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving clockwise");
    rclcpp::sleep_for(3s);
  }

  void move_counterclockwise() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {-1.0, 1.0, 1.0, -1.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Moving counterclockwise");
  }

  void stop() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {0.0, 0.0, 0.0, 0.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Stop");
  }

  void publish_wheel_velocities() {
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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelocities>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}