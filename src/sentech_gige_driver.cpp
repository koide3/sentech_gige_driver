#include <iostream>
#include <ros/ros.h>

#include <StApi_TL.h>
#include <StApi_GUI.h>

#include <opencv2/opencv.hpp>

using namespace StApi;


int main(int argc, char** argv) {
  ros::init(argc, argv, "sentech_gige_driver");

  CStApiAutoInit stapi_auto_init;
  CIStSystemPtr ist_system(CreateIStSystem(StSystemVendor_Sentech));
  CIStDevicePtr ist_device(ist_system->CreateFirstIStDevice());
  std::cout << "device:" << ist_device->GetIStDeviceInfo()->GetDisplayName() << std::endl;

  CIStDataStreamPtr ist_data_stream(ist_device->CreateIStDataStream(0));
  ist_data_stream->StartAcquisition();
  ist_device->AcquisitionStart();

  cv::Mat input_mat;
  cv::Mat bgr_image;

  while (ist_data_stream->IsGrabbing() && ros::ok()) {
    CIStStreamBufferPtr ist_stream_buffer(ist_data_stream->RetrieveBuffer(1000));

    if(ist_stream_buffer->GetIStStreamBufferInfo()->IsImagePresent()) {
      IStImage* ist_image = ist_stream_buffer->GetIStImage();
      const auto pixel_format = ist_image->GetImagePixelFormat();
      IStPixelFormatInfo* const pixel_format_info = GetIStPixelFormatInfo(pixel_format);

      if(pixel_format_info->IsMono() || pixel_format_info->IsBayer()) {
        const size_t width = ist_image->GetImageWidth();
        const size_t height = ist_image->GetImageHeight();
        int input_type = CV_8UC1;
        if(8 < pixel_format_info->GetEachComponentTotalBitCount()) {
          input_type = CV_16UC1;
        }

        input_mat.create(height, width, input_type);

        const size_t buffer_size = input_mat.rows * input_mat.cols * input_mat.elemSize() * input_mat.channels();
        memcpy(input_mat.ptr(0), ist_image->GetImageBuffer(), buffer_size);

        if (pixel_format_info->IsBayer()) {
          int convert_code = 0;
          switch (pixel_format_info->IsBayer()) {
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
            cv::cvtColor(input_mat, bgr_image, convert_code);
          }
        }
      }

      cv::imshow("image", input_mat);
      cv::waitKey(1);

    } else {
      std::cerr << "no image" << std::endl;
    }

    ros::spinOnce();
  }

  ist_device->AcquisitionStop();
  ist_data_stream->StopAcquisition();

  return 0;
}