// 哨兵决策：巡逻 → 血量低 / 最后 60 秒 → 回补给点待命
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "mini_sentry_interfaces/action/go_to_point.hpp"
#include "mini_sentry_interfaces/msg/game_status.hpp"
#include "mini_sentry_interfaces/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using GameStatus = mini_sentry_interfaces::msg::GameStatus;
using GoToPoint = mini_sentry_interfaces::action::GoToPoint;
using SetMode = mini_sentry_interfaces::srv::SetMode;

class SentryBrain : public rclcpp::Node
{
public:
  using GoalHandleGoTo = rclcpp_action::ClientGoalHandle<GoToPoint>;

  SentryBrain()
  : Node("sentry_brain")
  {
    retreat_hp_ = this->declare_parameter<int>("retreat_hp", 150);
    defense_time_ = this->declare_parameter<double>("defense_time", 60.0);
    supply_x_ = this->declare_parameter<double>("supply_x", 0.0);
    supply_y_ = this->declare_parameter<double>("supply_y", 0.0);
    patrol_ = this->declare_parameter<std::vector<double>>(
      "patrol_waypoints", {1.0, 1.0, 4.0, 1.0, 4.0, 3.0, 1.0, 3.0});

    if (patrol_.size() < 2 || patrol_.size() % 2 != 0) {
      RCLCPP_ERROR(this->get_logger(), "patrol_waypoints 需要偶数个（x,y 成对）");
      patrol_ = {1.0, 1.0, 4.0, 1.0, 4.0, 3.0, 1.0, 3.0};
    }

    status_sub_ = this->create_subscription<GameStatus>(
      "game_status", rclcpp::QoS(10),
      [this](const GameStatus::SharedPtr msg) { status_ = msg; });

    mode_srv_ = this->create_service<SetMode>(
      "set_mode",
      std::bind(&SentryBrain::on_set_mode, this,
        std::placeholders::_1, std::placeholders::_2));

    action_client_ = rclcpp_action::create_client<GoToPoint>(this, "go_to_point");
    timer_ = this->create_wall_timer(100ms, std::bind(&SentryBrain::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "sentry_brain 启动：retreat_hp=%d，defense_time=%.0f s，%zu 个航点",
      retreat_hp_, defense_time_, patrol_.size() / 2);
  }

private:
  // ---------------- 作业 1 的核心：最后 60 秒回防 ----------------
  void tick()
  {
    // 回防改道：先取消巡逻目标，等 0.3 s 让取消落到服务端，再发 (0,0)。
    // 不等这一下会出现"取消所有目标"把刚发的回防目标也取消掉，然后重发一次的抖动。
    if (pending_supply_goal_) {
      if (send_delay_ticks_ > 0) {
        --send_delay_ticks_;
        return;
      }
      pending_supply_goal_ = false;
      if (!goal_active_ && !retreat_done_) {
        go_to(supply_x_, supply_y_);
      }
      return;
    }

    if (!status_) {
      return;
    }
    if (status_->game_progress != GameStatus::PROGRESS_RUNNING) {
      return;
    }

    const int remain = static_cast<int>(status_->stage_remain_time);
    const bool hp_low = static_cast<int>(status_->current_hp) < retreat_hp_;
    const bool last_minute = remain < static_cast<int>(defense_time_);

    // 规则 A（作业 1）：最后 defense_time 秒 → 回补给点待命（不管血量）
    // 规则 B：血量低于阈值 → 也回补给点
    if (last_minute || hp_low) {
      if (mode_ != SetMode::Request::RETREAT) {
        if (last_minute) {
          RCLCPP_INFO(this->get_logger(),
            "剩余 %d s < %.0f s，进入回防：回补给点 (%.1f, %.1f)",
            remain, defense_time_, supply_x_, supply_y_);
        } else {
          RCLCPP_WARN(this->get_logger(),
            "血量 %u < %d，撤退回补给点", status_->current_hp, retreat_hp_);
        }
        mode_ = SetMode::Request::RETREAT;
        auto_retreat_ = true;  // 自动触发：条件恢复后才自动切回巡逻
        // 巡逻目标还在跑就先取消，马上改道。
        // 注意别用 async_cancel_all_goals()：取消请求到服务端要几十毫秒，
        // 服务端会把"取消所有目标"应用到刚发出去的回防目标上（表现是回防目标被 CANCELED、
        // 然后又被重发一次）。这里精确取消这一个目标，并且让新目标晚 0.3 s 再发。
        if (goal_active_) {
          if (goal_handle_) {
            action_client_->async_cancel_goal(goal_handle_);
          } else {
            action_client_->async_cancel_all_goals();
          }
          ++goal_seq_;            // 立刻作废旧目标的序号，它的 CANCELED 结果回来时直接被忽略
          goal_active_ = false;
          pending_supply_goal_ = true;
          send_delay_ticks_ = 3;  // 3 个 tick = 0.3 s
        }
      }
      // 没在跑目标就（重新）发一次，服务端没起来也不会漏掉
      if (!goal_active_ && !retreat_done_) {
        if (send_delay_ticks_ > 0) {
          --send_delay_ticks_;
          return;
        }
        go_to(supply_x_, supply_y_);
      }
      return;
    }

    // 规则 C：血量/时间都恢复正常，从待命重新出来巡逻（只对自动回防生效，
    //        手动 service 切过来的"回防/待命"不该被 tick 一秒后改回去）
    if (mode_ == SetMode::Request::RETREAT && auto_retreat_) {
      RCLCPP_INFO(this->get_logger(), "血量 %u、剩余 %d s，恢复正常巡逻",
        status_->current_hp, remain);
      mode_ = SetMode::Request::PATROL;
      retreat_done_ = false;
      auto_retreat_ = false;
    }

    // 规则 D：正常巡逻
    if (mode_ != SetMode::Request::PATROL || goal_active_) {
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "goto_server 还没起来，等待中…");
      return;
    }
    go_to(patrol_[2 * waypoint_index_], patrol_[2 * waypoint_index_ + 1]);
  }

  void go_to(double x, double y)
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "goto_server 未就绪，本次目标丢弃");
      return;
    }
    if (goal_active_) {
      return;
    }

    GoToPoint::Goal goal;
    goal.x = x;
    goal.y = y;

    // 每个目标一个序号：取消后旧目标的回调还会回来，只有"当前序号"的回调才允许改状态，
    // 否则 tick 会被旧回调骗着重复发目标（演示里会出现两次"已回到补给点"）
    const uint64_t my_seq = ++goal_seq_;

    auto options = rclcpp_action::Client<GoToPoint>::SendGoalOptions();
    options.goal_response_callback =
      [this, my_seq](const GoalHandleGoTo::SharedPtr & handle) {
        if (my_seq != goal_seq_) {
          return;  // 已经改道，这个响应过时了
        }
        if (!handle) {
          goal_active_ = false;
          RCLCPP_WARN(this->get_logger(), "目标被 goto_server 拒绝");
          return;
        }
        goal_handle_ = handle;
      };
    options.feedback_callback =
      [this](GoalHandleGoTo::SharedPtr, const std::shared_ptr<const GoToPoint::Feedback> fb) {
        RCLCPP_DEBUG(this->get_logger(), "剩余 %.2f m", fb->distance_remaining);
      };
    options.result_callback =
      [this, my_seq, x, y](const GoalHandleGoTo::WrappedResult & r) {
        if (my_seq != goal_seq_) {
          return;  // 被取消的旧目标，结果不参与决策
        }
        goal_active_ = false;
        goal_handle_.reset();
        if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
          if (std::abs(x - supply_x_) < 1e-6 && std::abs(y - supply_y_) < 1e-6) {
            retreat_done_ = true;
            RCLCPP_INFO(this->get_logger(), "已回到补给点，原地待命");
          } else {
            waypoint_index_ = (waypoint_index_ + 1) % (patrol_.size() / 2);
            RCLCPP_INFO(this->get_logger(), "到达航点，下一个是 #%zu",
              waypoint_index_ + 1);
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "目标未完成（code=%d）",
            static_cast<int>(r.code));
        }
      };

    goal_active_ = true;  // 先置位，避免 tick 重复发目标
    action_client_->async_send_goal(goal, options);
  }

  void on_set_mode(const SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr res)
  {
    if (req->mode != SetMode::Request::PATROL &&
      req->mode != SetMode::Request::HOLD &&
      req->mode != SetMode::Request::RETREAT)
    {
      res->success = false;
      res->message = "未知模式";
      return;
    }

    mode_ = req->mode;
    retreat_done_ = false;
    auto_retreat_ = false;  // 手动切模式，不让 tick 自动改回去
    if (mode_ == SetMode::Request::HOLD) {
      if (goal_active_) {
        if (goal_handle_) {
          action_client_->async_cancel_goal(goal_handle_);
        } else {
          action_client_->async_cancel_all_goals();
        }
      }
      goal_active_ = false;
      ++goal_seq_;
      retreat_done_ = true;  // 手动待命，不要再自动去补给点
    } else if (mode_ == SetMode::Request::RETREAT) {
      if (goal_active_) {
        if (goal_handle_) {
          action_client_->async_cancel_goal(goal_handle_);
        } else {
          action_client_->async_cancel_all_goals();
        }
        ++goal_seq_;
        goal_active_ = false;
        pending_supply_goal_ = true;
        send_delay_ticks_ = 3;   // 由 tick 在 0.3 s 后发 (0,0)，避开取消请求的时序
      } else {
        go_to(supply_x_, supply_y_);
      }
    }

    res->success = true;
    res->message = std::string("已切换到 ") + mode_name();
    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }

  std::string mode_name() const
  {
    switch (mode_) {
      case SetMode::Request::PATROL: return "巡逻";
      case SetMode::Request::HOLD: return "待命";
      case SetMode::Request::RETREAT: return "回防";
      default: return "未知";
    }
  }

  int retreat_hp_{150};
  double defense_time_{60.0};
  double supply_x_{0.0};
  double supply_y_{0.0};
  std::vector<double> patrol_;
  std::size_t waypoint_index_{0};
  uint8_t mode_{SetMode::Request::PATROL};
  bool goal_active_{false};
  bool retreat_done_{false};
  bool pending_supply_goal_{false};
  bool auto_retreat_{false};
  int send_delay_ticks_{0};
  uint64_t goal_seq_{0};

  GameStatus::SharedPtr status_;
  GoalHandleGoTo::SharedPtr goal_handle_;
  rclcpp::Subscription<GameStatus>::SharedPtr status_sub_;
  rclcpp::Service<SetMode>::SharedPtr mode_srv_;
  rclcpp_action::Client<GoToPoint>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SentryBrain>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
