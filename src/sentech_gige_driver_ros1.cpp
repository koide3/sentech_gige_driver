#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sentech_gige_driver.hpp>

class SentechGigeDriverNode : public SentechGigeDriver {
public:
  SentechGigeDriverNode() : nh("~"), it(nh) {
    frame_id = "camera";
    SentechGigeDriverParams params;

    nh.getParam("frame_id", frame_id);
    nh.getParam("camera_id", params.camera_id);
    nh.getParam("enable_ptp", params.enable_ptp);
    nh.getParam("auto_gain", params.auto_gain);
    nh.getParam("gain", params.gain);
    nh.getParam("auto_exposure", params.auto_exposure);
    nh.getParam("exposure", params.exposure);
    nh.getParam("exposure_min", params.exposure_min);
    nh.getParam("exposure_max", params.exposure_max);
    nh.getParam("fps", params.fps);

    init(params);

    camera_info.reset(new camera_info_manager::CameraInfoManager(nh, camera_name));

    camera_pub = it.advertiseCamera("image", 10);
    ptp_status_pub = nh.advertise<std_msgs::String>("ptp_status", 10);
  }

  virtual ~SentechGigeDriverNode() override {}

  virtual void publish_image(std::uint64_t stamp, const cv::Mat& bgr_image) override {
    std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp.sec = stamp / 1000000000;
    header.stamp.nsec = stamp % 1000000000;

    auto info_msg = camera_info->getCameraInfo();
    info_msg.header = header;

    auto image_msg = cv_bridge::CvImage(header, "bgr8", bgr_image).toImageMsg();

    camera_pub.publish(*image_msg, info_msg);
  }

  virtual void publish_ptp_status(const std::string& status) override {
    std_msgs::String msg;
    msg.data = status;
    ptp_status_pub.publish(msg);
  }

private:
  std::string frame_id;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::CameraPublisher camera_pub;
  ros::Publisher ptp_status_pub;

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info;
};

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "sentech_gige_driver");

    auto node = std::make_shared<SentechGigeDriverNode>();

    ros::spin();
  } catch (const GenICam::GenericException& e) {
    std::cerr << "exception: " << e.GetDescription() << std::endl;
  }

  return 0;
}