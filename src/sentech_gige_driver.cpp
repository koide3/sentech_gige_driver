#include <iostream>
#include <ros/ros.h>

#include <StApi_TL.h>
#include <StApi_GUI.h>

#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace StApi;
using namespace GenApi;

class SentechGigeDriver {
public:
  SentechGigeDriver() {
    ROS_INFO_STREAM("Opening camera");
    ist_system = CreateIStSystem(StSystemVendor_Sentech);
    ist_device = ist_system->CreateFirstIStDevice();

    const auto device_name = ist_device->GetIStDeviceInfo()->GetDisplayName();
    const std::string camera_name = device_name.c_str();
    ROS_INFO_STREAM("device:" << camera_name);

    ist_data_stream = ist_device->CreateIStDataStream(0);
    ist_data_stream->StartAcquisition();
    ist_device->AcquisitionStart();

    enable_ptp();

    // ROS related
    nh = ros::NodeHandle("~");

    frame_id = nh.param<std::string>("frame_id", "camera");

    it.reset(new image_transport::ImageTransport(nh));
    camera_info.reset(new camera_info_manager::CameraInfoManager(nh, camera_name));

    image_pub = it->advertise("image", 10);
    camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
    ptp_status_pub = nh.advertise<std_msgs::String>("ptp_status", 1);

    ptp_status_timer = nh.createWallTimer(ros::WallDuration(2.0), &SentechGigeDriver::publish_ptp_status, this);
  }

  ~SentechGigeDriver() {
    ist_device->AcquisitionStop();
    ist_data_stream->StopAcquisition();
  }

  void spin() {
    std::string encoding = "mono8";
    cv::Mat input_mat;
    cv::Mat bgr_image;

    while (ist_data_stream->IsGrabbing() && ros::ok()) {
      CIStStreamBufferPtr ist_stream_buffer(ist_data_stream->RetrieveBuffer(1000));
      if (ist_stream_buffer->GetIStStreamBufferInfo()->IsImagePresent()) {
        IStImage* ist_image = ist_stream_buffer->GetIStImage();
        const auto pixel_format = ist_image->GetImagePixelFormat();
        IStPixelFormatInfo* const pixel_format_info = GetIStPixelFormatInfo(pixel_format);

        if (pixel_format_info->IsMono() || pixel_format_info->IsBayer()) {
          const size_t width = ist_image->GetImageWidth();
          const size_t height = ist_image->GetImageHeight();
          int input_type = CV_8UC1;
          if (8 < pixel_format_info->GetEachComponentTotalBitCount()) {
            input_type = CV_16UC1;
          }

          input_mat.create(height, width, input_type);

          const size_t buffer_size = input_mat.rows * input_mat.cols * input_mat.elemSize() * input_mat.channels();
          memcpy(input_mat.ptr(0), ist_image->GetImageBuffer(), buffer_size);

          if (pixel_format_info->IsBayer()) {
            int convert_code = 0;
            switch (pixel_format_info->GetPixelColorFilter()) {
              case (StPixelColorFilter_BayerRG):
                convert_code = cv::COLOR_BayerRG2BGR;
                break;
              case (StPixelColorFilter_BayerGR):
                convert_code = cv::COLOR_BayerGR2BGR;
                break;
              case (StPixelColorFilter_BayerGB):
                convert_code = cv::COLOR_BayerGB2BGR;
                break;
              case (StPixelColorFilter_BayerBG):
                convert_code = cv::COLOR_BayerBG2BGR;
                break;
            }

            if (convert_code != 0) {
              encoding = "bgr8";
              cv::cvtColor(input_mat, bgr_image, convert_code);
              input_mat = bgr_image;
            }
          }
        }

        if (image_pub.getNumSubscribers()) {
          std_msgs::Header header;
          header.frame_id = frame_id;

          const auto stamp = ist_stream_buffer->GetIStStreamBufferInfo()->GetTimestamp();
          header.stamp.sec = stamp / 1000000000;
          header.stamp.nsec = stamp % 1000000000;

          cv_bridge::CvImage cv_image(header, encoding, input_mat);
          image_pub.publish(cv_image.toImageMsg());

          auto info = camera_info->getCameraInfo();
          info.header = header;
          camera_info_pub.publish(info);
        }
      } else {
        ROS_WARN_STREAM("No image data");
      }

      ros::spinOnce();
    }
  }

private:
  void enable_ptp() {
    ROS_INFO_STREAM("Enabling PTP");

    auto node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("PtpEnable");
    if (!GenApi::IsWritable(node)) {
      ROS_WARN_STREAM("PTP node is not writable!!");
      return;
    }

    GenApi::CBooleanPtr value(node);
    value->SetValue(true);

    ROS_INFO_STREAM("PTP enabled");
  }

  void publish_ptp_status(const ros::WallTimerEvent& e) {
    auto latch_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("PtpDataSetLatch");
    if(!GenApi::IsAvailable(latch_node)) {
      ROS_WARN_STREAM("PTP latch node is not available");
    }

    GenApi::CCommandPtr command(latch_node);
    command->Execute();

    auto node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("PtpStatus");
    if (!GenApi::IsReadable(node)) {
      ROS_WARN_STREAM("PTP status is not readable!!");
      return;
    }

    GenApi::CEnumerationPtr value(node);
    const int valuei = value->GetIntValue();
    GenApi::CEnumEntryPtr entry(value->GetCurrentEntry());

    std::stringstream sst;
    sst << "Status:" << entry->GetSymbolic().c_str();

    std_msgs::String str;
    str.data = sst.str();

    ptp_status_pub.publish(str);
  }

private:
  CStApiAutoInit stapi_auto_init;
  CIStSystemPtr ist_system;
  CIStDevicePtr ist_device;

  CIStDataStreamPtr ist_data_stream;

  ros::NodeHandle nh;
  std::unique_ptr<image_transport::ImageTransport> it;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info;

  image_transport::Publisher image_pub;
  ros::Publisher camera_info_pub;
  ros::Publisher ptp_status_pub;

  ros::WallTimer ptp_status_timer;

  std::string frame_id;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sentech_gige_driver");

  SentechGigeDriver driver;
  driver.spin();

  return 0;
}