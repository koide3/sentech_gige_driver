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
    exposure = 15000.0;
    exposure_min = 10.0;
    exposure_max = 50000.0;

    fps = 100.0;
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
  SentechGigeDriver() : initialized(false), stopped(false) {}

  virtual ~SentechGigeDriver() {
    if (initialized && !stopped) {
      stop();
    }
  }

  void init(const SentechGigeDriverParams& params) {
    using namespace StApi;
    using namespace GenApi;

    this->params = params;

    std::cout << "Opening camera" << std::endl;
    ist_system = CreateIStSystem(StSystemVendor_Sentech);
    ist_device = ist_system->CreateFirstIStDevice();

    const auto device_name = ist_device->GetIStDeviceInfo()->GetDisplayName();
    camera_name = device_name.c_str();
    std::cout << "Device:" << camera_name << std::endl;

    ist_data_stream = ist_device->CreateIStDataStream(0);
    RegisterCallback(ist_data_stream, *this, &SentechGigeDriver::image_callback, (void*)nullptr);

    ist_data_stream->StartAcquisition();
    ist_device->AcquisitionStart();
    initialized = true;

    std::cout << "---" << std::endl;

    // Gain
    if (params.auto_gain) {
      if (!set_param<std::string>("GainAuto", "Continuous")) {
        std::cerr << "warning: Use digial gain because analogue gain is not supported" << std::endl;
        set_param<std::string>("GainSelector", "DigitalAll");
        set_param<std::string>("GainAuto", "Continuous");
      }
    } else {
      set_param<std::string>("GainAuto", "Off");
      set_param<double>("Gain", params.gain);
    }

    // Exposure
    if (params.auto_exposure) {
      set_param<std::string>("ExposureAuto", "Continuous");
      set_param<double>("ExposureAutoLimitMin", params.exposure_min);
      set_param<double>("ExposureAutoLimitMax", params.exposure_max);
    } else {
      set_param<std::string>("ExposureAuto", "Off");
      set_param<double>("ExposureTime", params.exposure);
    }

    // FPS
    set_param<double>("AcquisitionFrameRate", params.fps);

    // PTP
    set_param<bool>("PtpEnable", params.enable_ptp);
    if (params.enable_ptp) {
      execute_command("PtpDataSetLatch");
    }

    std::cout << "---" << std::endl;
  }

  void stop() {
    std::cout << "Stopping Acquisition" << std::endl;
    ist_device->AcquisitionStop();
    ist_data_stream->StopAcquisition();
    stopped = true;
  }

  virtual void publish_image(std::uint64_t stamp, const cv::Mat& bgr_image) = 0;
  virtual void publish_ptp_status(const std::string& status) = 0;

protected:
  GenApi::INode* get_read_node(const std::string& node_name) {
    auto node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode(node_name.c_str());

    if (!GenApi::IsAvailable(node)) {
      std::cerr << "warning: " << node_name << " is not available!!" << std::endl;
      return nullptr;
    }

    if (!GenApi::IsReadable(node)) {
      std::cerr << "warning: " << node_name << " is not readable!!" << std::endl;
      return nullptr;
    }

    return node;
  }

  GenApi::INode* get_write_node(const std::string& node_name) {
    auto node = ist_device->GetRemoteIStPort()->GetINodeMap()->GetNode(node_name.c_str());

    if (!GenApi::IsAvailable(node)) {
      std::cerr << "warning: " << node_name << " is not available!!" << std::endl;
      return nullptr;
    }

    if (!GenApi::IsWritable(node)) {
      std::cerr << "warning: " << node_name << " is not writable!!" << std::endl;
      return nullptr;
    }

    return node;
  }

  bool set_param(GenApi::INode* node, bool value) {
    GenApi::CBooleanPtr value_ptr(node);
    value_ptr->SetValue(value);
    std::cout << " to " << value << std::endl;
    return true;
  }

  bool set_param(GenApi::INode* node, double value) {
    GenApi::CFloatPtr value_ptr(node);
    double min = value_ptr->GetMin();
    double max = value_ptr->GetMax();
    value = std::min(max, std::max(min, value));
    value_ptr->SetValue(value);
    std::cout << " to " << value << " (" << min << " ~ " << max << ")" << std::endl;
    return true;
  }

  bool set_param(GenApi::INode* node, int value) {
    GenApi::CIntegerPtr value_ptr(node);
    int min = value_ptr->GetMin();
    int max = value_ptr->GetMax();
    value = std::min(max, std::max(min, value));
    value_ptr->SetValue(value);
    std::cout << " to " << value << " (" << min << " ~ " << max << ")" << std::endl;
    return true;
  }

  bool set_param(GenApi::INode* node, const std::string& value) {
    GenApi::CEnumerationPtr value_ptr(node);
    GenApi::IEnumEntry* entry = value_ptr->GetEntryByName(value.c_str());

    if (!entry) {
      std::cerr << "warning: invalid entry " << value << std::endl;
      return false;
    }

    value_ptr->SetIntValue(entry->GetValue());
    std::cout << " to " << value << std::endl;

    return true;
  }

  template <typename T>
  bool set_param(const std::string& node_name, const T& value) {
    auto node = get_write_node(node_name);
    if (!node) {
      return false;
    }

    std::cout << "Set " << node_name;
    return set_param(node, value);
  }

  bool get_bool(const std::string& node_name) {
    auto node = get_read_node(node_name);
    if (!node) {
      return false;
    }

    GenApi::CBooleanPtr value_ptr(node);
    return value_ptr->GetValue();
  }

  int get_int(const std::string& node_name) {
    auto node = get_read_node(node_name);
    if (!node) {
      return 0;
    }

    GenApi::CIntegerPtr value_ptr(node);
    return value_ptr->GetValue();
  }

  std::string get_enum(const std::string& node_name) {
    auto node = get_read_node(node_name);
    if (!node) {
      return "";
    }

    GenApi::CEnumerationPtr value_ptr(node);
    GenApi::CEnumEntryPtr entry(value_ptr->GetCurrentEntry());
    return entry->GetSymbolic().c_str();
  }

  void execute_command(const std::string& node_name) {
    auto node = get_write_node(node_name);
    if (!node) {
      return;
    }

    GenApi::CCommandPtr command(node);
    command->Execute();
  }

  void image_callback(StApi::IStCallbackParamBase* ist_callback_param_base, void* context) {
    using namespace StApi;
    using namespace GenApi;

    if (ist_callback_param_base->GetCallbackType() == StCallbackType_GenTLEvent_DataStreamNewBuffer) {
      IStCallbackParamGenTLEventNewBuffer* new_buffer = dynamic_cast<IStCallbackParamGenTLEventNewBuffer*>(ist_callback_param_base);

      try {
        IStDataStream* ist_data_stream = new_buffer->GetIStDataStream();
        CIStStreamBufferPtr ist_stream_buffer = ist_data_stream->RetrieveBuffer(0);

        const IStStreamBufferInfo* stream_buffer_info = ist_stream_buffer->GetIStStreamBufferInfo();

        if (stream_buffer_info->IsImagePresent()) {
          IStImage* ist_image = ist_stream_buffer->GetIStImage();
          const auto pixel_format = ist_image->GetImagePixelFormat();
          const IStPixelFormatInfo* pixel_format_info = GetIStPixelFormatInfo(pixel_format);
          const std::uint64_t stamp = stream_buffer_info->GetTimestamp();

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
                default:
                  std::cerr << "error: unknown bayer pattern " << static_cast<int>(pixel_format_info->GetPixelColorFilter()) << std::endl;
                case (StPixelColorFilter_BayerRG):
                  convert_code = cv::COLOR_BayerRG2RGB;
                  break;
                case (StPixelColorFilter_BayerGR):
                  convert_code = cv::COLOR_BayerGR2RGB;
                  break;
                case (StPixelColorFilter_BayerGB):
                  convert_code = cv::COLOR_BayerGB2RGB;
                  break;
                case (StPixelColorFilter_BayerBG):
                  convert_code = cv::COLOR_BayerBG2RGB;
                  break;
              }

              if (convert_code != 0) {
                cv::cvtColor(input_mat, bgr_image, convert_code);
                publish_image(stamp, bgr_image);
              }
            }
          }
        } else {
          std::cerr << "image does not exist" << std::endl;
        }

      } catch (const GenICam::GenericException& e) {
        std::cerr << "exception:" << e.GetDescription() << std::endl;
      }
    }

    if (params.enable_ptp && (std::chrono::high_resolution_clock::now() - last_ptp_time) > std::chrono::seconds(2)) {
      const std::string status = get_enum("PtpStatus");
      execute_command("PtpDataSetLatch");
      publish_ptp_status(status);
      last_ptp_time = std::chrono::high_resolution_clock::now();
    }
  }

protected:
  SentechGigeDriverParams params;
  std::string camera_name;

  bool initialized;
  bool stopped;

  StApi::CStApiAutoInit auto_init;
  StApi::CIStSystemPtr ist_system;
  StApi::CIStDevicePtr ist_device;
  StApi::CIStDataStreamPtr ist_data_stream;

  cv::Mat input_mat;
  cv::Mat bgr_image;

  std::chrono::high_resolution_clock::time_point last_ptp_time;
};