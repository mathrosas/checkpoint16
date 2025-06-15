#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <eigen3/Eigen/Dense>

using std::placeholders::_1;

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    RCLCPP_INFO(get_logger(), "Initialized kinematic model publisher node.");
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10, std::bind(&KinematicModel::callback, this, _1));
    // Robot geometry parameters (half distances)
    w_ = 0.26969 / 2.0;
    l_ = 0.17000 / 2.0;
    r_ = 0.10000 / 2.0;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

  float w_, l_, r_;

  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
      RCLCPP_WARN(get_logger(), "Expected 4 wheel speeds, got %zu",
                  msg->data.size());
      return;
    }

    // Holonomic drive matrix H_ (4x3): maps wheel velocities [ω, vx, vy] to
    // wheel speeds
    Eigen::Matrix<float, 4, 3> H_;
    H_ << -l_ - w_, 1, -1, l_ + w_, 1, 1, l_ + w_, 1, -1, -l_ - w_, 1, 1;
    // Scale by wheel radius
    H_ /= r_;

    // Wheel speeds vector U (4x1)
    Eigen::Matrix<float, 4, 1> U;
    U << msg->data[0], msg->data[1], msg->data[2], msg->data[3];

    // Compute pseudoinverse of H_ via SVD: H_pinv (3x4)
    Eigen::JacobiSVD<Eigen::Matrix<float, 4, 3>> svd(
        H_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &S = svd.singularValues();
    Eigen::Matrix<float, 3, 4> S_pinv = Eigen::Matrix<float, 3, 4>::Zero();
    const float tol = 1e-6f;
    for (int i = 0; i < S.size(); ++i) {
      if (S(i) > tol) {
        S_pinv(i, i) = 1.0 / S(i);
      }
    }
    Eigen::Matrix<float, 3, 4> H_pinv =
        svd.matrixV() * S_pinv * svd.matrixU().transpose();

    // Compute wheel velocities: [ω, vx, vy] = H_pinv * U
    Eigen::Matrix<float, 3, 1> wheel_vel = H_pinv * U;

    // Convert to Twist message
    geometry_msgs::msg::Twist twist;
    twist.angular.z = wheel_vel(0);
    twist.linear.x = wheel_vel(1);
    twist.linear.y = wheel_vel(2);

    RCLCPP_INFO(get_logger(),
                "Computed wheel velocities ω: %.3f, vx: %.3f, vy: %.3f",
                wheel_vel(0), wheel_vel(1), wheel_vel(2));

    // Publish to /cmd_vel
    pub_->publish(twist);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}
