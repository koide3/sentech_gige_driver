#pragma once

#include <StApi_TL.h>
#include <StApi_GUI.h>
#include <opencv2/opencv.hpp>

struct SentechGigeDriverParams {
  SentechGigeDriverParams() {
    camera_id = 0;
    enable_ptp = false;

    auto_gain = true;
    gain = 22.0;

    auto_exposure = true;
    exposure = 14878.0;
    exposure_min = 10.0;
    exposure_max = 50000.0;

    fps = 0.0;
  }

  int camera_id;
  bool enable_ptp;

  bool auto_gain;
  double gain;

  bool auto_exposure;
  double exposure;
  double exposure_min;
  double exposure_max;

  double fps;
};

class SentechGigeDriver {
public:
  SentechGigeDriver() {}
  virtual ~SentechGigeDriver() {}

  void init(const SentechGigeDriverParams& params) {
    using namespace StApi;
    using namespace GenApi;

    this->params = params;

    std::cout << "Opening camera" << std::endl;
    ist_system = CreateIStSystem(StSystemVendor_Sentech);
    ist_device = ist_system->CreateFirstIStDevice();

    const auto device_name = ist_device->GetIStDeviceInfo()->GetDisplayName();
    camera_name = device_name.c_str();
    std::cout << "device:" << camera_name;

    ist_data_stream = ist_device->CreateIStDataStream(params.camera_id);
    ist_data_stream->StartAcquisition();
    ist_device->AcquisitionStart();

    if (params.enable_ptp) {
      enable_ptp();
    }

    set_gain(params.auto_gain, params.gain);
    set_exposure(params.auto_exposure, params.exposure, params.exposure_min, params.exposure_max);

    set_fps(params.fps);
  }

  void stop() {
    std::cout << "Stopping Acquisition" << std::endl;
    try {
      ist_device->AcquisitionStop();
      ist_data_stream->StopAcquisition();
    } catch (const GenICam::GenericException& e) {
      std::cerr << "exception:" << e.GetDescription() << std::endl;
    }
  }

  std::pair<std::uint64_t, cv::Mat> get_image() {
    using namespace StApi;
    using namespace GenApi;

    if (!ist_data_stream->IsGrabbing()) {
      return std::make_pair(0, cv::Mat());
    }

    std::string encoding = "mono8";
    cv::Mat input_mat;
    cv::Mat bgr_image;

    CIStStreamBufferPtr ist_stream_buffer(ist_data_stream->RetrieveBuffer(1000));
    if (ist_stream_buffer->GetIStStreamBufferInfo()->IsImagePresent()) {
      IStImage* ist_image = ist_stream_buffer->GetIStImage();
      const auto pixel_format = ist_image->GetImagePixelFormat();
      IStPixelFormatInfo* const pixel_format_info = GetIStPixelFormatInfo(pixel_format);
      const auto stamp = ist_stream_buffer->GetIStStreamBufferInfo()->GetTimestamp();

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
            encoding = "rgb8";
            cv::cvtColor(input_mat, bgr_image, convert_code);
            input_mat = bgr_image;
          }
        }
      }

      return std::make_pair(stamp, bgr_image);
    }

    return std::make_pair(0, cv::Mat());
  }

protected:
  void set_fps(double fps) {
    std::cout << "Setting FPS" << std::endl;

    auto node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("AcquisitionFrameRate");
    if (!GenApi::IsWritable(node)) {
      std::cout << "AcquisitionFrameRate is not writable!!" << std::endl;
      return;
    }

    GenApi::CFloatPtr value(node);
    const double min_fps = value->GetMin();
    const double max_fps = value->GetMax();
    std::cout << "FPS range:" << min_fps << " ~ " << max_fps << std::endl;

    if (fps <= 0.0) {
      fps = max_fps;
    }
    fps = std::max(min_fps, std::min(max_fps, fps));

    std::cout << "Set FPS:" << fps << std::endl;
    value->SetValue(fps);
  }

  void set_gain(bool auto_gain, double gain) {
    std::cout << "Setting gain" << std::endl;

    auto gain_auto_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("GainAuto");
    if (!GenApi::IsAvailable(gain_auto_node) || !GenApi::IsWritable(gain_auto_node)) {
      std::cout << "GainAuto is not available or writable!!" << std::endl;
      std::cout << "Switching to digital gain!!" << std::endl;

      auto gain_selector_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("GainSelector");
      if (!GenApi::IsAvailable(gain_selector_node) || !GenApi::IsWritable(gain_selector_node)) {
        std::cout << "GainSelector is not available or writable!!" << std::endl;
      } else {
        GenApi::CEnumerationPtr item(gain_selector_node);
        item->SetIntValue(item->GetEntryByName("DigitalAll")->GetValue());

        gain_auto_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("GainAuto");
      }
    }

    if (GenApi::IsAvailable(gain_auto_node) && GenApi::IsWritable(gain_auto_node)) {
      GenApi::CEnumerationPtr value(gain_auto_node);

      if (auto_gain) {
        std::cout << "Auto gain" << std::endl;
        value->SetIntValue(value->GetEntryByName("Continuous")->GetValue());
      } else {
        std::cout << "Manual gain" << std::endl;
        value->SetIntValue(value->GetEntryByName("Off")->GetValue());
      }
    } else {
      std::cout << "Failed to set GainAuto" << std::endl;
    }

    if (!auto_gain && gain > 0.0) {
      auto gain_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("Gain");
      GenApi::CFloatPtr gain_value(gain_node);
      const double gain_min = gain_value->GetMin();
      const double gain_max = gain_value->GetMax();

      std::cout << "Gain range:" << gain_min << " ~ " << gain_max << std::endl;
      gain = std::max(gain_min, std::min(gain_max, gain));

      std::cout << "Set Gain:" << gain << std::endl;
      gain_value->SetValue(gain);
    }
  }

  void set_exposure(bool auto_exposure, double exposure, double exposure_min, double exposure_max) {
    std::cout << "Setting exposure" << std::endl;

    auto node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("ExposureAuto");
    if (!GenApi::IsAvailable(node)) {
      std::cout << "ExposureAuto node is not available" << std::endl;
    }

    GenApi::CEnumerationPtr value(node);

    if (auto_exposure) {
      std::cout << "Auto exposure" << std::endl;
      GenApi::CEnumEntryPtr continous = value->GetEntryByName("Continuous");
      value->SetIntValue(continous->GetValue());

      auto exposure_min_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("ExposureAutoLimitMin");
      auto exposure_max_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("ExposureAutoLimitMax");

      if (!GenApi::IsWritable(exposure_min_node) || !GenApi::IsWritable(exposure_max_node)) {
        std::cerr << "Auto exposure limits are not writable!!" << std::endl;
      } else {
        GenApi::CFloatPtr(exposure_min_node)->SetValue(exposure_min);
        GenApi::CFloatPtr(exposure_max_node)->SetValue(exposure_max);

        std::cout << "Auto exposure limit:" << exposure_min << " ~ " << exposure_max << std::endl;
      }

    } else {
      std::cout << "Manual exposure" << std::endl;
      GenApi::CEnumEntryPtr off = value->GetEntryByName("Off");
      value->SetIntValue(off->GetValue());

      if (exposure > 0.0) {
        auto exposure_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("ExposureTime");
        GenApi::CFloatPtr exposure_value(exposure_node);
        const double exposure_min = exposure_value->GetMin();
        const double exposure_max = exposure_value->GetMax();

        std::cout << "Exposure range:" << exposure_min << " ~ " << exposure_max << std::endl;
        exposure = std::max(exposure_min, std::min(exposure_max, exposure));

        std::cout << "Set Exposure:" << exposure << std::endl;
        exposure_value->SetValue(exposure);
      }
    }
  }

  void enable_ptp() {
    std::cout << "Enabling PTP" << std::endl;

    auto node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("PtpEnable");
    if (!GenApi::IsWritable(node)) {
      std::cout << "PTP node is not writable!!" << std::endl;
      return;
    }

    GenApi::CBooleanPtr value(node);
    value->SetValue(true);

    std::cout << "PTP enabled" << std::endl;

    std::cout << "Latching PTP" << std::endl;
    auto latch_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("PtpDataSetLatch");
    if (!GenApi::IsAvailable(latch_node)) {
      std::cout << "PTP latch node is not available" << std::endl;
    }

    GenApi::CCommandPtr command(latch_node);
    command->Execute();
  }

  std::string get_ptp_status() {
    if (!params.enable_ptp) {
      return "Status:N/A";
    }

    auto node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("PtpStatus");
    if (!GenApi::IsReadable(node)) {
      std::cout << "PTP status is not readable!!" << std::endl;
      return "Status:Error";
    }

    GenApi::CEnumerationPtr value(node);
    const int valuei = value->GetIntValue();
    GenApi::CEnumEntryPtr entry(value->GetCurrentEntry());

    std::string status = entry->GetSymbolic().c_str();

    std::stringstream sst;
    sst << "Status:" << status;

    if (status == "Disabled") {
      std::cout << "Latching PTP" << std::endl;
      auto latch_node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode("PtpDataSetLatch");
      if (!GenApi::IsAvailable(latch_node)) {
        std::cout << "PTP latch node is not available" << std::endl;
      }

      GenApi::CCommandPtr command(latch_node);
      command->Execute();
    }

    return sst.str();
  }

protected:
  SentechGigeDriverParams params;
  std::string camera_name;

  StApi::CStApiAutoInit stapi_auto_init;
  StApi::CIStSystemPtr ist_system;
  StApi::CIStDevicePtr ist_device;

  StApi::CIStDataStreamPtr ist_data_stream;
};