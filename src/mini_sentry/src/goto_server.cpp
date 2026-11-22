// 到点动作服务端：接受目标，用 /cmd_vel 把底盘开过去，持续反馈剩余距离
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "mini_sentry_interfaces/action/go_to_point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class GoToServer : public rclcpp::Node
{
public:
  using GoToPoint = mini_sentry_interfaces::action::GoToPoint;
  using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoToPoint>;

  GoToServer()
  : Node("goto_server")
  {
    speed_ = this->declare_parameter<double>("speed", 1.0);
    tolerance_ = this->declare_parameter<double>("tolerance", 0.15);
    max_time_ = this->declare_parameter<double>("max_time", 60.0);
    control_rate_ = this->declare_parameter<double>("control_rate", 20.0);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        const auto & q = msg->pose.pose.orientation;
        yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        have_odom_ = true;
      });

    action_server_ = rclcpp_action::create_server<GoToPoint>(
      this, "go_to_point",
      std::bind(&GoToServer::handle_goal, this, _1, _2),
      std::bind(&GoToServer::handle_cancel, this, _1),
      std::bind(&GoToServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "goto_server 就绪：/go_to_point（最快 %.2f m/s）", speed_);
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoToPoint::Goal> goal)
  {
    if (!std::isfinite(goal->x) || !std::isfinite(goal->y)) {
      RCLCPP_WARN(this->get_logger(), "目标点非法，拒绝");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "收到目标 (%.2f, %.2f)", goal->x, goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGoTo>)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoTo> goal_handle)
  {
    // 执行要占住线程，不能放在执行器的回调里
    std::thread{std::bind(&GoToServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoTo> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPoint::Feedback>();
    auto result = std::make_shared<GoToPoint::Result>();
    rclcpp::Rate rate(control_rate_);
    const auto start = this->now();

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stop();
        result->reached = false;
        result->total_time = (this->now() - start).seconds();
        goal_handle->canceled(result);
        return;
      }

      double cx = 0.0, cy = 0.0, cyaw = 0.0;
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        cx = x_;
        cy = y_;
        cyaw = yaw_;
      }

      const double dx = goal->x - cx;
      const double dy = goal->y - cy;
      const double dist = std::hypot(dx, dy);

      if (dist < tolerance_) {
        stop();
        result->reached = true;
        result->total_time = (this->now() - start).seconds();
        RCLCPP_INFO(this->get_logger(), "到达 (%.2f, %.2f)，用时 %.1f s",
          goal->x, goal->y, result->total_time);
        goal_handle->succeed(result);
        return;
      }

      if ((this->now() - start).seconds() > max_time_) {
        stop();
        result->reached = false;
        result->total_time = (this->now() - start).seconds();
        RCLCPP_WARN(this->get_logger(), "超时 %.0f s，放弃目标", max_time_);
        goal_handle->abort(result);
        return;
      }

        // 全向底盘：世界系速度转回车体系
      const double v = std::min(speed_, 0.2 + std::min(dist, 1.0));
      const double wx = v * dx / dist;
      const double wy = v * dy / dist;
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = std::cos(cyaw) * wx + std::sin(cyaw) * wy;
      cmd.linear.y = -std::sin(cyaw) * wx + std::cos(cyaw) * wy;
      cmd_pub_->publish(cmd);

      feedback->distance_remaining = dist;
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }
  }

  void stop()
  {
    geometry_msgs::msg::Twist zero;
    cmd_pub_->publish(zero);
  }

  double speed_{1.0};
  double tolerance_{0.15};
  double max_time_{60.0};
  double control_rate_{20.0};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  bool have_odom_{false};
  std::mutex pose_mutex_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<GoToPoint>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
