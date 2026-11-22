// 作业 4：订阅 /image，画个框 + 时间戳，再发到 /image_debug
#include <algorithm>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"   // Lyrical+ 把 .h 改成了 .hpp
#else
#include "cv_bridge/cv_bridge.h"     // Humble / Jazzy
#endif
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class ImageDebug : public rclcpp::Node
{
public:
  ImageDebug()
  : Node("image_debug")
  {
    box_w_ = this->declare_parameter<int>("box_width", 240);
    box_h_ = this->declare_parameter<int>("box_height", 180);
    queue_size_ = this->declare_parameter<int>("queue_size", 1);

    // 相机是 BEST_EFFORT 的，订阅端跟着降级；只留最新一帧，处理慢也不积压
    auto qos = rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(queue_size_));
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", qos, std::bind(&ImageDebug::on_image, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_debug", qos);

    RCLCPP_INFO(this->get_logger(), "image_debug 就绪：/image -> /image_debug（框 %d×%d）",
      box_w_, box_h_);
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      // toCvShare 不复制；但下面要改像素，必须先 clone，别动原始 buffer
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败：%s", e.what());
      return;
    }

    cv::Mat img = cv_ptr->image.clone();
    const int w = img.cols;
    const int h = img.rows;

    cv::Rect box((w - box_w_) / 2, (h - box_h_) / 2,
      std::max(1, std::min(box_w_, w)), std::max(1, std::min(box_h_, h)));
    cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2);

    char text[128];
    std::snprintf(text, sizeof(text), "frame %u", static_cast<unsigned>(msg->header.stamp.sec));
    cv::putText(img, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 0), 2);

    // 用 CvImage 保留原 header（时间戳和 frame_id 不能丢）
    cv_bridge::CvImage out;
    out.header = msg->header;
    out.encoding = sensor_msgs::image_encodings::BGR8;
    out.image = img;
    pub_->publish(*out.toImageMsg());
  }

  int box_w_{240};
  int box_h_{180};
  int queue_size_{1};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageDebug>());
  rclcpp::shutdown();
  return 0;
}
