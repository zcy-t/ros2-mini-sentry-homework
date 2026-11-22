// 作业 2：速度写成参数，支持 ros2 param set / rqt_reconfigure 在线修改
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CmdTest : public rclcpp::Node
{
public:
  CmdTest()
  : Node("cmd_test")
  {
    linear_x_ = this->declare_parameter<double>("linear_x", 0.6);
    angular_z_ = this->declare_parameter<double>("angular_z", 0.8);
    max_speed_ = this->declare_parameter<double>("max_speed", 2.0);

    // 构造时读到的值只在这里用一次；之后想改必须靠下面的回调
    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&CmdTest::on_set_parameters, this, _1));

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    timer_ = this->create_wall_timer(100ms, std::bind(&CmdTest::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "cmd_test 启动：linear_x=%.2f m/s, angular_z=%.2f rad/s（画圆半径约 %.2f m）",
      linear_x_, angular_z_, linear_x_ / std::max(std::abs(angular_z_), 0.01));
  }

private:
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // 先整体校验，全部通过才写进去，避免改一半
    double new_linear = linear_x_;
    double new_angular = angular_z_;
    double new_max = max_speed_;

    for (const auto & p : params) {
      if (p.get_name() == "linear_x") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = "linear_x 必须是 double";
          return result;
        }
        if (std::abs(p.as_double()) > new_max) {
          result.successful = false;
          result.reason = "linear_x 超限，|v| 不能大于 max_speed=" +
            std::to_string(new_max);
          return result;
        }
        new_linear = p.as_double();
      } else if (p.get_name() == "angular_z") {
        if (std::abs(p.as_double()) > 3.0) {
          result.successful = false;
          result.reason = "angular_z 超限，|w| 不能大于 3.0";
          return result;
        }
        new_angular = p.as_double();
      } else if (p.get_name() == "max_speed") {
        if (p.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "max_speed 必须大于 0";
          return result;
        }
        new_max = p.as_double();
      }
    }

    linear_x_ = new_linear;
    angular_z_ = new_angular;
    max_speed_ = new_max;
    RCLCPP_INFO(this->get_logger(), "参数已生效：v=%.2f m/s, w=%.2f rad/s",
      linear_x_, angular_z_);
    return result;
  }

  void tick()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x_;
    cmd.angular.z = angular_z_;
    pub_->publish(cmd);
  }

  double linear_x_{0.6};
  double angular_z_{0.8};
  double max_speed_{2.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdTest>());
  rclcpp::shutdown();
  return 0;
}
