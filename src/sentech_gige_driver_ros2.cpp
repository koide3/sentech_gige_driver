#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <sentech_gige_driver.hpp>

class SentechGigeDriverROS2 : public rclcpp::Node, public SentechGigeDriver {
public:
  SentechGigeDriverROS2(rclcpp::NodeOptions& options) : rclcpp::Node("sentech_gige_driver", options) {
    frame_id = "camera";
    SentechGigeDriverParams params;

    this->declare_parameter<std::string>("frame_id", "camera");
    this->declare_parameter<int>("camera_id", params.camera_id);
    this->declare_parameter<bool>("enable_ptp", params.enable_ptp);
    this->declare_parameter<bool>("auto_gain", params.auto_gain);
    this->declare_parameter<double>("gain", params.gain);
    this->declare_parameter<bool>("auto_exposure", params.auto_exposure);
    this->declare_parameter<double>("exposure", params.exposure);
    this->declare_parameter<double>("exposure_min", params.exposure_min);
    this->declare_parameter<double>("exposure_max", params.exposure_max);

    this->get_parameter("frame_id", frame_id);
    this->get_parameter("camera_id", params.camera_id);
    this->get_parameter("enable_ptp", params.enable_ptp);
    this->get_parameter("auto_gain", params.auto_gain);
    this->get_parameter("gain", params.gain);
    this->get_parameter("auto_exposure", params.auto_exposure);
    this->get_parameter("exposure", params.exposure);
    this->get_parameter("exposure_min", params.exposure_min);
    this->get_parameter("exposure_max", params.exposure_max);

    init(params);

    camera_info.reset(new camera_info_manager::CameraInfoManager(this, camera_name));

    image_pub = image_transport::create_camera_publisher(this, "image");
    ptp_status_pub = this->create_publisher<std_msgs::msg::String>("ptp_status", 10);

    timer = this->create_wall_timer(std::chrono::milliseconds(5), [this] { timer_callback(); });
    if (params.enable_ptp) {
      ptp_status_timer = this->create_wall_timer(std::chrono::seconds(2), [this] { ptp_status_timer_callback(); });
    }
  }

  ~SentechGigeDriverROS2() {  //
    stop();
  }

  void timer_callback() {
    const auto stamp_image = get_image();
    const auto& stamp = stamp_image.first;
    const auto& image = stamp_image.second;

    if (!image.data) {
      return;
    }

    std_msgs::msg::Header header;
    header.frame_id = frame_id;
    header.stamp.sec = stamp / 1000000000;
    header.stamp.nanosec = stamp % 1000000000;

    auto info_msg = camera_info->getCameraInfo();
    auto image_msg = cv_bridge::CvImage(header, "rgb8", image).toImageMsg();

    image_pub.publish(*image_msg, info_msg);
  }

  void ptp_status_timer_callback() {
    if (!ptp_status_pub->get_subscription_count()) {
      return;
    }

    std_msgs::msg::String status_msg;
    status_msg.data = get_ptp_status();
    ptp_status_pub->publish(status_msg);
  }

private:
  std::string frame_id;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr ptp_status_timer;

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info;

  image_transport::CameraPublisher image_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ptp_status_pub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SentechGigeDriverROS2>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}