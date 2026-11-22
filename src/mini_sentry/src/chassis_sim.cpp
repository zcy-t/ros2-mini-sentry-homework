// 底盘仿真：收 /cmd_vel，积分出位姿，发 /odom 和 odom -> base_link 的 TF
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#if __has_include("tf2_ros/transform_broadcaster.hpp")
#include "tf2_ros/transform_broadcaster.hpp"   // Lyrical+
#else
#include "tf2_ros/transform_broadcaster.h"     // Humble/Jazzy
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;

class ChassisSim : public rclcpp::Node
{
public:
  ChassisSim()
  : Node("chassis_sim")
  {
    max_speed_ = this->declare_parameter<double>("max_speed", 2.0);
    rate_ = this->declare_parameter<double>("rate", 50.0);
    cmd_timeout_ = this->declare_parameter<double>("cmd_timeout", 0.5);

    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&ChassisSim::on_set_parameters, this, _1));

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&ChassisSim::on_cmd, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ChassisSim::update, this));

    last_update_ = this->now();
    last_cmd_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "chassis_sim 启动：max_speed=%.2f m/s, rate=%.1f Hz",
      max_speed_, rate_);
  }

private:
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      if (p.get_name() == "max_speed") {
        if (p.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "max_speed 必须大于 0";
        } else {
          max_speed_ = p.as_double();
          RCLCPP_INFO(this->get_logger(), "max_speed 已更新为 %.2f m/s", max_speed_);
        }
      }
      // rate / cmd_timeout 只在启动时生效：定时器已按旧频率创建
    }
    return result;
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_ = *msg;
    last_cmd_time_ = this->now();
  }

  void update()
  {
    const rclcpp::Time t = this->now();
    const double dt = (t - last_update_).seconds();
    last_update_ = t;
    if (dt <= 0.0) {
      return;
    }

    // 超时停车：0.5 s 没收到新指令就清零，上游崩了车也不会一直冲
    double vx = 0.0, vy = 0.0, wz = 0.0;
    if ((t - last_cmd_time_).seconds() <= cmd_timeout_) {
      vx = std::clamp(cmd_.linear.x, -max_speed_, max_speed_);
      vy = std::clamp(cmd_.linear.y, -max_speed_, max_speed_);
      wz = std::clamp(cmd_.angular.z, -2.0, 2.0);
    }

    x_ += (vx * std::cos(yaw_) - vy * std::sin(yaw_)) * dt;
    y_ += (vx * std::sin(yaw_) + vy * std::cos(yaw_)) * dt;
    yaw_ += wz * dt;

    publish_odom(t, vx, vy, wz);
    publish_tf(t);
  }

  void publish_odom(const rclcpp::Time & stamp, double vx, double vy, double wz)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    // 只绕 z 轴转：q = (0,0,sin(yaw/2),cos(yaw/2))，不依赖 tf2::Quaternion
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;
    odom_pub_->publish(odom);
  }

  void publish_tf(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = std::sin(yaw_ / 2.0);
    tf.transform.rotation.w = std::cos(yaw_ / 2.0);
    tf_broadcaster_->sendTransform(tf);
  }

  double max_speed_{2.0};
  double rate_{50.0};
  double cmd_timeout_{0.5};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time last_update_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisSim>());
  rclcpp::shutdown();
  return 0;
}
