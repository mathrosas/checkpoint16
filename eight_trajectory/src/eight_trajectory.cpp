// src/absolute_motion.cpp

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "eigen3/Eigen/Dense"

using namespace std::chrono_literals;

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("absolute_motion"), phi_(0.0) {
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectory::odom_callback, this,
                  std::placeholders::_1));
    // sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/rosbot_xl_base_controller/odom", 10,
    //     std::bind(&EightTrajectory::odom_callback, this,
    //               std::placeholders::_1));
  }

  void run() {
    double dphi, dx, dy;
    double goal_x = 0.0, goal_y = 0.0, goal_phi = 0.0;

    while (pub_->get_subscription_count() == 0) {
      rclcpp::sleep_for(100ms);
    }

    for (auto [rel_phi, rel_x, rel_y] : motions_) {
      goal_phi += rel_phi;
      goal_x += rel_x;
      goal_y += rel_y;

      //   dphi = M_PI * target_phi / 180.0 - phi_;
      dphi = goal_phi - phi_;
      dx = goal_x - x_;
      dy = goal_y - y_;

      while (std::hypot(dx, dy) > pos_tol || std::abs(dphi) > ang_tol) {
        {
          auto [wz, vx, vy] = velocity2twist(dphi, dx, dy);
          auto wheels = twist2wheels(wz, vx, vy);

          std_msgs::msg::Float32MultiArray msg;
          msg.data = wheels;
          pub_->publish(msg);

          rclcpp::spin_some(shared_from_this()); // Process callbacks
          rclcpp::sleep_for(std::chrono::milliseconds(
              25)); // odometry is published at 10-12Hz

          // debug print
          RCLCPP_INFO(get_logger(),
                      "err pos=%.3f (dx=%.3f,dy=%.3f), err ang=%.3f",
                      std::hypot(dx, dy), dx, dy, dphi);

          //   dphi = M_PI * target_phi / 180.0 - phi_;
          dphi = goal_phi - phi_;
          dx = goal_x - x_;
          dy = goal_y - y_;
        }
      }
      stop();
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  double x_, y_, phi_;

  double pos_tol = 0.03;
  double ang_tol = 0.03;

  // Define the sequence of (phi, x, y) motions
  std::vector<std::tuple<double, double, double>> motions_{
      {0.0, 1.0, -1.0},     {0.0, 1.0, 1.0},       {0.0, 1.0, 1.0},
      {-1.5708, 1.0, -1.0}, {-1.5708, -1.0, -1.0}, {0.0, -1.0, 1.0},
      {0.0, -1.0, 1.0},     {0.0, -1.0, -1.0}};

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    // Extract yaw (φ) from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    phi_ = yaw;
  }

  std::tuple<double, double, double> velocity2twist(double dphi, double dx,
                                                    double dy) {
    // Build rotation matrix R(φ)
    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, std::cos(phi_), std::sin(phi_), 0, -std::sin(phi_),
        std::cos(phi_);

    Eigen::Vector3d v(dphi, dx, dy);
    Eigen::Vector3d twist = R * v;

    // twist[0]=wz, twist[1]=vx, twist[2]=vy
    return std::make_tuple(twist(0), twist(1), twist(2));
  }

  std::vector<float> twist2wheels(double wz, double vx, double vy) {
    // Robot geometry
    double w_ = 0.26969 / 2.0; // half wheelbase
    double l_ = 0.17000 / 2.0; // half track width
    double r_ = 0.10000 / 2.0; // wheel radius

    // H matrix (4×3)
    Eigen::Matrix<double, 4, 3> H;
    H << -l_ - w_, 1, -1, l_ + w_, 1, 1, l_ + w_, 1, -1, -l_ - w_, 1, 1;
    H /= r_;

    Eigen::Vector3d twist(wz, vx, vy);
    Eigen::Matrix<double, 4, 1> u = H * twist;

    // cast each wheel speed to float
    return {static_cast<float>(u(0, 0)), static_cast<float>(u(1, 0)),
            static_cast<float>(u(2, 0)), static_cast<float>(u(3, 0))};
  }

  void stop() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {0.0, 0.0, 0.0, 0.0};
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Stop");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EightTrajectory>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
