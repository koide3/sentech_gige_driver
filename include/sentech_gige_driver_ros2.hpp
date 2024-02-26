#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <sentech_gige_driver.hpp>

namespace sentech_gige_driver {

class SentechGigeDriverNode : public rclcpp::Node, public SentechGigeDriver {
public:
  SentechGigeDriverNode(rclcpp::NodeOptions options) : rclcpp::Node("sentech_gige_driver", options) {
    frame_id = "camera";
    SentechGigeDriverParams params;

    this->declare_parameter<std::string>("frame_id", "camera");
    this->declare_parameter<int>("camera_id", params.camera_id);
    this->declare_parameter<bool>("enable_ptp", params.enable_ptp);
    this->declare_parameter<bool>("auto_gain", params.auto_gain);
    this->declare_parameter<int>("luminance_target", params.luminance_target);
    this->declare_parameter<double>("gain", params.gain);
    this->declare_parameter<bool>("auto_exposure", params.auto_exposure);
    this->declare_parameter<double>("exposure", params.exposure);
    this->declare_parameter<double>("exposure_min", params.exposure_min);
    this->declare_parameter<double>("exposure_max", params.exposure_max);
    this->declare_parameter<double>("fps", params.fps);

    this->get_parameter("frame_id", frame_id);
    this->get_parameter("camera_id", params.camera_id);
    this->get_parameter("enable_ptp", params.enable_ptp);
    this->get_parameter("auto_gain", params.auto_gain);
    this->get_parameter("luminance_target", params.luminance_target);
    this->get_parameter("gain", params.gain);
    this->get_parameter("auto_exposure", params.auto_exposure);
    this->get_parameter("exposure", params.exposure);
    this->get_parameter("exposure_min", params.exposure_min);
    this->get_parameter("exposure_max", params.exposure_max);
    this->get_parameter("fps", params.fps);

    init(params);
    camera_info.reset(new camera_info_manager::CameraInfoManager(this, camera_name));

    image_pub = image_transport::create_camera_publisher(this, "image", rmw_qos_profile_sensor_data);
    ptp_status_pub = this->create_publisher<std_msgs::msg::String>("ptp_status", 10);

    start();
  }

  virtual ~SentechGigeDriverNode() override {}

  virtual void publish_image(std::uint64_t stamp, const cv::Mat& bgr_image) override {
    std_msgs::msg::Header header;
    header.frame_id = frame_id;
    header.stamp.sec = stamp / 1000000000;
    header.stamp.nanosec = stamp % 1000000000;

    auto info_msg = camera_info->getCameraInfo();
    info_msg.header = header;

    auto image_msg = cv_bridge::CvImage(header, "bgr8", bgr_image).toImageMsg();

    image_pub.publish(*image_msg, info_msg);
  }

  virtual void publish_ptp_status(const std::string& status) override {
    std_msgs::msg::String msg;
    msg.data = status;
    ptp_status_pub->publish(msg);
  }

private:
  std::string frame_id;

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info;

  image_transport::CameraPublisher image_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ptp_status_pub;
};

}