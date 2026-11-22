// TF 练习：把 livox_frame 下的一个点换到 base_link / odom，看看坐标树对不对
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#if __has_include("tf2_ros/buffer.hpp")
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#else
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#endif

using namespace std::chrono_literals;

class TfDemo : public rclcpp::Node
{
public:
  TfDemo()
  : Node("tf_demo")
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    source_frame_ = this->declare_parameter<std::string>("source_frame", "livox_frame");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(1s, std::bind(&TfDemo::report, this));
  }

private:
  void report()
  {
    geometry_msgs::msg::PointStamped in;
    in.header.frame_id = source_frame_;
    in.header.stamp = this->now();
    in.point.x = 0.0;
    in.point.y = 0.0;
    in.point.z = 1.0;  // 雷达坐标系上方 1 m 的一个点

    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame_, source_frame_, this->now(), rclcpp::Duration::from_seconds(0.2));

      geometry_msgs::msg::PointStamped out;
      tf2::doTransform(in, out, tf);
      RCLCPP_INFO(this->get_logger(),
        "%s 下 (%.2f, %.2f, %.2f) -> %s 下 (%.2f, %.2f, %.2f)",
        source_frame_.c_str(), in.point.x, in.point.y, in.point.z,
        target_frame_.c_str(), out.point.x, out.point.y, out.point.z);

      // 顺便查一下车在世界系的位姿（自瞄需要的是这个坐标系）
      const auto odom_tf = tf_buffer_->lookupTransform(
        "odom", target_frame_, this->now(), rclcpp::Duration::from_seconds(0.1));
      const double yaw = std::atan2(
        2.0 * (odom_tf.transform.rotation.w * odom_tf.transform.rotation.z +
        odom_tf.transform.rotation.x * odom_tf.transform.rotation.y),
        1.0 - 2.0 * (odom_tf.transform.rotation.y * odom_tf.transform.rotation.y +
        odom_tf.transform.rotation.z * odom_tf.transform.rotation.z));
      RCLCPP_INFO(this->get_logger(), "odom -> %s：x=%.2f y=%.2f yaw=%.1f°",
        target_frame_.c_str(),
        odom_tf.transform.translation.x, odom_tf.transform.translation.y,
        yaw * 180.0 / M_PI);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "查 TF 失败：%s", ex.what());
    }
  }

  std::string target_frame_;
  std::string source_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfDemo>());
  rclcpp::shutdown();
  return 0;
}
