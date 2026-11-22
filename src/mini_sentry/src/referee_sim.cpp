// 裁判系统仿真：按时间推进比赛状态，发布 /game_status
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "mini_sentry_interfaces/msg/game_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using GameStatus = mini_sentry_interfaces::msg::GameStatus;

class RefereeSim : public rclcpp::Node
{
public:
  RefereeSim()
  : Node("referee_sim")
  {
    max_hp_ = this->declare_parameter<int>("max_hp", 400);
    match_duration_ = this->declare_parameter<double>("match_duration", 420.0);
    start_delay_ = this->declare_parameter<double>("start_delay", 3.0);
    // 模拟被打：剩余时间小于 hp_drop_time 时血量掉到 hp_after_damage
    simulate_damage_ = this->declare_parameter<bool>("simulate_damage", true);
    hp_drop_time_ = this->declare_parameter<double>("hp_drop_time", 150.0);
    hp_after_damage_ = this->declare_parameter<int>("hp_after_damage", 140);

    pub_ = this->create_publisher<GameStatus>("game_status", rclcpp::QoS(10));
    start_ = this->now();
    timer_ = this->create_wall_timer(200ms, std::bind(&RefereeSim::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "referee_sim 启动：时长 %.0f s，%.0f s 后开始，simulate_damage=%s",
      match_duration_, start_delay_, simulate_damage_ ? "true" : "false");
  }

private:
  void tick()
  {
    // 每帧重新读参数：这样 ros2 param set / rqt_reconfigure 改时长、掉血开关能立即生效
    max_hp_ = this->get_parameter("max_hp").as_int();
    match_duration_ = this->get_parameter("match_duration").as_double();
    simulate_damage_ = this->get_parameter("simulate_damage").as_bool();
    hp_drop_time_ = this->get_parameter("hp_drop_time").as_double();
    hp_after_damage_ = this->get_parameter("hp_after_damage").as_int();

    const double elapsed = (this->now() - start_).seconds();
    GameStatus msg;
    msg.maximum_hp = static_cast<uint16_t>(max_hp_);

    if (elapsed < start_delay_) {
      msg.game_progress = GameStatus::PROGRESS_NOT_STARTED;
      msg.stage_remain_time = static_cast<uint16_t>(std::max(0.0, match_duration_));
      msg.current_hp = static_cast<uint16_t>(max_hp_);
      pub_->publish(msg);
      return;
    }

    const double remain = match_duration_ - (elapsed - start_delay_);
    msg.game_progress = remain > 0.0 ? GameStatus::PROGRESS_RUNNING
                                     : GameStatus::PROGRESS_FINISHED;
    msg.stage_remain_time = static_cast<uint16_t>(std::max(0.0, std::round(remain)));

    int hp = max_hp_;
    if (simulate_damage_ && remain < hp_drop_time_) {
      hp = hp_after_damage_;
    }
    msg.current_hp = static_cast<uint16_t>(std::max(0, std::min(hp, max_hp_)));
    pub_->publish(msg);
  }

  int max_hp_{400};
  double match_duration_{420.0};
  double start_delay_{3.0};
  bool simulate_damage_{true};
  double hp_drop_time_{150.0};
  int hp_after_damage_{140};
  rclcpp::Time start_;
  rclcpp::Publisher<GameStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RefereeSim>());
  rclcpp::shutdown();
  return 0;
}
