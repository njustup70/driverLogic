#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "MvCameraControl.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

namespace hik_camera_ros2_driver
{
class HikCameraRos2DriverNode : public rclcpp::Node
{
public:
  explicit HikCameraRos2DriverNode(const rclcpp::NodeOptions & options)
  : Node("hik_camera_ros2_driver", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraRos2DriverNode!");

    initializeCamera();
    declareParameters();
    startCamera();

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraRos2DriverNode::dynamicParametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread(&HikCameraRos2DriverNode::captureLoop, this);
  }

  ~HikCameraRos2DriverNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraRos2DriverNode destroyed!");
  }

private:
  bool initializeCamera()
  {
    MV_CC_DEVICE_INFO_LIST device_list;

    // enum device
    while (rclcpp::ok()) {
      n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      if (n_ret_ != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enumerate devices, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else if (device_list.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "No camera found, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else {
        RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);
        break;
      }
    }

    n_ret_ = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create camera handle!");
      return false;
    }

    n_ret_ = MV_CC_OpenDevice(camera_handle_);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera device!");
      return false;
    }

    // Get camera information
    n_ret_ = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get camera image info!");
      return false;
    }

    image_msg_.data.reserve(
      static_cast<size_t>(img_info_.nHeightMax) * static_cast<size_t>(img_info_.nWidthMax));

    return true;
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value = {};
    int status = MV_OK;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // ADC Bit Depth
    param_desc.description = "ADC Bit Depth";
    param_desc.additional_constraints = "Supported values: Bits_8, Bits_12";
    std::string adc_bit_depth = this->declare_parameter("adc_bit_depth", "", param_desc);
    if (!adc_bit_depth.empty()) {
      MVCC_ENUMVALUE adc_bit_depth_value = {};
      status = MV_CC_GetEnumValue(camera_handle_, "ADCBitDepth", &adc_bit_depth_value);
      if (status == MV_OK) {
        status = MV_CC_SetEnumValueByString(camera_handle_, "ADCBitDepth", adc_bit_depth.c_str());
        if (status == MV_OK) {
          RCLCPP_INFO(this->get_logger(), "ADC Bit Depth set to %s", adc_bit_depth.c_str());
        } else {
          RCLCPP_WARN(
            this->get_logger(), "Failed to set ADC Bit Depth to %s, status = %s",
            adc_bit_depth.c_str(), statusToString(status).c_str());
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(), "ADCBitDepth is not available on this camera, status = %s",
          statusToString(status).c_str());
      }
    }

    // Pixel format
    param_desc.description = "Pixel Format";
    std::string pixel_format = this->declare_parameter("pixel_format", "Mono8", param_desc);
    configurePixelFormat(pixel_format);

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    if (status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to set ExposureTime to %f, status = %s", exposure_time,
        statusToString(status).c_str());
    }

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    status = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    if (status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to set Gain to %f, status = %s", gain,
        statusToString(status).c_str());
    }

    // Acquisition frame rate
    param_desc.description = "Acquisition frame rate in Hz";
    MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double acquisition_frame_rate =
      this->declare_parameter("acquisition_frame_rate", 165.0, param_desc);
    status = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
    if (status != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to enable AcquisitionFrameRate, status = %s",
        statusToString(status).c_str());
    }
    status = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", acquisition_frame_rate);
    if (status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Acquisition frame rate: %f", acquisition_frame_rate);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to set AcquisitionFrameRate to %f, status = %s",
        acquisition_frame_rate, statusToString(status).c_str());
    }
  }

  void startCamera()
  {
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    camera_name_ = this->declare_parameter("camera_name", "camera");
    frame_id_ = this->declare_parameter("frame_id", camera_name_ + "_optical_frame");
    camera_topic_ = this->declare_parameter("camera_topic", camera_name_ + "/image");
    int image_node_num = this->declare_parameter("image_node_num", 8);
    image_node_num = std::max(1, std::min(image_node_num, 30));

    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, camera_topic_, qos);

    int status = MV_CC_SetImageNodeNum(camera_handle_, static_cast<unsigned int>(image_node_num));
    if (status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "SDK image node number: %d", image_node_num);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to set image node number to %d, status = %s",
        image_node_num, statusToString(status).c_str());
    }

    status = MV_CC_StartGrabbing(camera_handle_);
    if (status != MV_OK) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to start grabbing, status = %s", statusToString(status).c_str());
    }

    // Load camera info
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter(
      "camera_info_url", "package://hik_camera_ros2_driver/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
  }

  void captureLoop()
  {
    MV_FRAME_OUT out_frame = {};
    RCLCPP_INFO(this->get_logger(), "Publishing image!");

    image_msg_.header.frame_id = frame_id_;
    image_msg_.encoding = "mono8";

    while (rclcpp::ok()) {
      n_ret_ = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
      if (MV_OK == n_ret_) {
        fail_count_ = 0;
        no_data_count_ = 0;
        image_msg_.header.stamp = this->now();
        image_msg_.height = out_frame.stFrameInfo.nHeight;
        image_msg_.width = out_frame.stFrameInfo.nWidth;
        image_msg_.step = out_frame.stFrameInfo.nWidth;
        image_msg_.data.resize(static_cast<size_t>(image_msg_.width) * image_msg_.height);

        if (fillMonoImage(out_frame)) {
          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);
        }

        MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

        static auto last_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 3) {
          MVCC_FLOATVALUE f_value;
          MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &f_value);
          RCLCPP_DEBUG(this->get_logger(), "ResultingFrameRate: %f Hz", f_value.fCurValue);
          last_log_time = now;
        }

        continue;
      }

      if (n_ret_ == static_cast<int>(MV_E_NODATA)) {
        no_data_count_++;
        warnBufferFailure("No image data from camera", n_ret_);
        if (no_data_count_ >= kMaxConsecutiveNoDataBeforeRestart) {
          RCLCPP_WARN(
            this->get_logger(), "No image data for %d consecutive waits; restarting stream.",
            no_data_count_);
          restartGrabbing();
          no_data_count_ = 0;
        }
      } else {
        fail_count_++;
        warnBufferFailure("Get image buffer failed", n_ret_);
        if (fail_count_ >= kMaxConsecutiveFailuresBeforeRestart) {
          RCLCPP_WARN(
            this->get_logger(), "Image buffer failed %d times consecutively; restarting stream.",
            fail_count_);
          restartGrabbing();
          fail_count_ = 0;
        }
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      const auto & type = param.get_type();
      const auto & name = param.get_name();
      int status = MV_OK;

      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (name == "gain") {
          status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        } else {
          result.successful = false;
          result.reason = "Unknown parameter: " + name;
          continue;
        }
      } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        if (name == "exposure_time") {
          status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        } else {
          result.successful = false;
          result.reason = "Unknown parameter: " + name;
          continue;
        }
      } else {
        result.successful = false;
        result.reason = "Unsupported parameter type for: " + name;
        continue;
      }

      if (status != MV_OK) {
        result.successful = false;
        result.reason = "Failed to set " + name + ", status = " + std::to_string(status);
      }
    }

    return result;
  }

  static std::string statusToString(int status)
  {
    std::ostringstream oss;
    oss << "0x" << std::hex << static_cast<unsigned int>(status);
    return oss.str();
  }

  static std::string pixelTypeToString(unsigned int pixel_type)
  {
    switch (pixel_type) {
      case PixelType_Gvsp_Mono8:
        return "Mono8";
      case PixelType_Gvsp_BayerGR8:
        return "BayerGR8";
      case PixelType_Gvsp_BayerRG8:
        return "BayerRG8";
      case PixelType_Gvsp_BayerGB8:
        return "BayerGB8";
      case PixelType_Gvsp_BayerBG8:
        return "BayerBG8";
      case PixelType_Gvsp_RGB8_Packed:
        return "RGB8Packed";
      case PixelType_Gvsp_BGR8_Packed:
        return "BGR8Packed";
      default: {
        std::ostringstream oss;
        oss << "0x" << std::hex << pixel_type;
        return oss.str();
      }
    }
  }

  static bool pixelTypeFromString(const std::string & name, unsigned int * pixel_type)
  {
    if (name == "Mono8") {
      *pixel_type = PixelType_Gvsp_Mono8;
    } else if (name == "BayerGR8") {
      *pixel_type = PixelType_Gvsp_BayerGR8;
    } else if (name == "BayerRG8") {
      *pixel_type = PixelType_Gvsp_BayerRG8;
    } else if (name == "BayerGB8") {
      *pixel_type = PixelType_Gvsp_BayerGB8;
    } else if (name == "BayerBG8") {
      *pixel_type = PixelType_Gvsp_BayerBG8;
    } else if (name == "RGB8Packed") {
      *pixel_type = PixelType_Gvsp_RGB8_Packed;
    } else if (name == "BGR8Packed") {
      *pixel_type = PixelType_Gvsp_BGR8_Packed;
    } else {
      return false;
    }
    return true;
  }

  static bool isDirectMono8PixelType(unsigned int pixel_type)
  {
    switch (pixel_type) {
      case PixelType_Gvsp_Mono8:
      case PixelType_Gvsp_BayerGR8:
      case PixelType_Gvsp_BayerRG8:
      case PixelType_Gvsp_BayerGB8:
      case PixelType_Gvsp_BayerBG8:
        return true;
      default:
        return false;
    }
  }

  static bool isPixelFormatSupported(const MVCC_ENUMVALUE & formats, unsigned int pixel_type)
  {
    for (unsigned int i = 0; i < formats.nSupportedNum && i < MV_MAX_XML_SYMBOLIC_NUM; ++i) {
      if (formats.nSupportValue[i] == pixel_type) {
        return true;
      }
    }
    return false;
  }

  static std::string supportedPixelFormatsToString(const MVCC_ENUMVALUE & formats)
  {
    std::ostringstream oss;
    for (unsigned int i = 0; i < formats.nSupportedNum && i < MV_MAX_XML_SYMBOLIC_NUM; ++i) {
      if (i != 0) {
        oss << ", ";
      }
      oss << pixelTypeToString(formats.nSupportValue[i]);
    }
    return oss.str();
  }

  bool pickDirectMonoPixelFormat(
    const MVCC_ENUMVALUE & formats, unsigned int requested_pixel_type,
    unsigned int * selected_pixel_type)
  {
    if (
      requested_pixel_type != PixelType_Gvsp_Undefined &&
      isPixelFormatSupported(formats, requested_pixel_type) &&
      isDirectMono8PixelType(requested_pixel_type))
    {
      *selected_pixel_type = requested_pixel_type;
      return true;
    }

    if (isPixelFormatSupported(formats, PixelType_Gvsp_Mono8)) {
      *selected_pixel_type = PixelType_Gvsp_Mono8;
      return true;
    }

    if (isDirectMono8PixelType(formats.nCurValue)) {
      *selected_pixel_type = formats.nCurValue;
      return true;
    }

    const unsigned int bayer_formats[] = {
      PixelType_Gvsp_BayerGB8, PixelType_Gvsp_BayerRG8, PixelType_Gvsp_BayerGR8,
      PixelType_Gvsp_BayerBG8};
    for (const auto bayer_format : bayer_formats) {
      if (isPixelFormatSupported(formats, bayer_format)) {
        *selected_pixel_type = bayer_format;
        return true;
      }
    }

    return false;
  }

  void configurePixelFormat(const std::string & requested_pixel_format)
  {
    MVCC_ENUMVALUE pixel_formats = {};
    int status = MV_CC_GetPixelFormat(camera_handle_, &pixel_formats);
    if (status != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to read supported PixelFormat values, status = %s",
        statusToString(status).c_str());
      return;
    }

    unsigned int requested_pixel_type = PixelType_Gvsp_Undefined;
    if (!requested_pixel_format.empty() && !pixelTypeFromString(
        requested_pixel_format, &requested_pixel_type))
    {
      RCLCPP_WARN(
        this->get_logger(), "Unknown requested PixelFormat '%s'; supported names include Mono8 "
        "and Bayer*8",
        requested_pixel_format.c_str());
    }

    unsigned int selected_pixel_type = pixel_formats.nCurValue;
    if (!pickDirectMonoPixelFormat(pixel_formats, requested_pixel_type, &selected_pixel_type)) {
      RCLCPP_WARN(
        this->get_logger(),
        "No directly publishable Mono8/Bayer8 PixelFormat found. Current format is %s; supported "
        "formats: %s. The driver will try SDK conversion to mono8.",
        pixelTypeToString(pixel_formats.nCurValue).c_str(),
        supportedPixelFormatsToString(pixel_formats).c_str());
    } else if (
      requested_pixel_type != PixelType_Gvsp_Undefined && selected_pixel_type != requested_pixel_type)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Requested PixelFormat %s is not supported for direct mono publishing; using %s instead. "
        "Supported formats: %s",
        requested_pixel_format.c_str(), pixelTypeToString(selected_pixel_type).c_str(),
        supportedPixelFormatsToString(pixel_formats).c_str());
    }

    if (selected_pixel_type != pixel_formats.nCurValue) {
      status = MV_CC_SetPixelFormat(camera_handle_, selected_pixel_type);
      if (status != MV_OK) {
        RCLCPP_WARN(
          this->get_logger(), "Failed to set PixelFormat to %s, status = %s. Keeping %s.",
          pixelTypeToString(selected_pixel_type).c_str(), statusToString(status).c_str(),
          pixelTypeToString(pixel_formats.nCurValue).c_str());
        selected_pixel_type = pixel_formats.nCurValue;
      }
    }

    RCLCPP_INFO(
      this->get_logger(), "Using camera PixelFormat %s and publishing sensor_msgs/Image mono8",
      pixelTypeToString(selected_pixel_type).c_str());
  }

  bool fillMonoImage(const MV_FRAME_OUT & out_frame)
  {
    const auto frame_pixel_type = static_cast<unsigned int>(out_frame.stFrameInfo.enPixelType);
    const size_t expected_size = static_cast<size_t>(image_msg_.width) * image_msg_.height;

    if (isDirectMono8PixelType(frame_pixel_type)) {
      if (static_cast<size_t>(out_frame.stFrameInfo.nFrameLen) < expected_size) {
        warnImageConversionFailure(
          "Frame payload is smaller than expected for mono8 publishing");
        fail_count_++;
        return false;
      }

      std::memcpy(image_msg_.data.data(), out_frame.pBufAddr, expected_size);
      return true;
    }

    MV_CC_PIXEL_CONVERT_PARAM convert_param = {};
    convert_param.nWidth = out_frame.stFrameInfo.nWidth;
    convert_param.nHeight = out_frame.stFrameInfo.nHeight;
    convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
    convert_param.pSrcData = out_frame.pBufAddr;
    convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
    convert_param.enDstPixelType = PixelType_Gvsp_Mono8;
    convert_param.pDstBuffer = image_msg_.data.data();
    convert_param.nDstBufferSize = static_cast<unsigned int>(image_msg_.data.size());

    const int status = MV_CC_ConvertPixelType(camera_handle_, &convert_param);
    if (status != MV_OK || convert_param.nDstLen < expected_size) {
      std::ostringstream oss;
      oss << "Failed to convert " << pixelTypeToString(frame_pixel_type)
          << " frame to mono8, status = " << statusToString(status);
      warnImageConversionFailure(oss.str());
      fail_count_++;
      return false;
    }

    return true;
  }

  void warnImageConversionFailure(const std::string & message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_image_warning_time_ >= std::chrono::seconds(1)) {
      RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
      last_image_warning_time_ = now;
    }
  }

  void warnBufferFailure(const std::string & message, int status)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_buffer_warning_time_ >= std::chrono::seconds(1)) {
      RCLCPP_WARN(
        this->get_logger(), "%s, status = %s", message.c_str(), statusToString(status).c_str());
      last_buffer_warning_time_ = now;
    }
  }

  bool restartGrabbing()
  {
    const int stop_status = MV_CC_StopGrabbing(camera_handle_);
    if (stop_status != MV_OK && stop_status != static_cast<int>(MV_E_CALLORDER)) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to stop grabbing during recovery, status = %s",
        statusToString(stop_status).c_str());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const int start_status = MV_CC_StartGrabbing(camera_handle_);
    if (start_status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Camera stream restarted.");
      return true;
    }
    if (start_status == static_cast<int>(MV_E_CALLORDER)) {
      RCLCPP_WARN(
        this->get_logger(),
        "StartGrabbing returned call-order error during recovery; stream may already be active.");
      return true;
    }

    RCLCPP_ERROR(
      this->get_logger(), "Failed to restart camera stream, status = %s",
      statusToString(start_status).c_str());
    return false;
  }

  void * camera_handle_ = nullptr;
  int n_ret_ = MV_OK;
  MV_IMAGE_BASIC_INFO img_info_;

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::string camera_name_;
  std::string frame_id_;
  std::string camera_topic_;

  std::thread capture_thread_;
  int fail_count_ = 0;
  int no_data_count_ = 0;
  static constexpr int kMaxConsecutiveNoDataBeforeRestart = 10;
  static constexpr int kMaxConsecutiveFailuresBeforeRestart = 5;
  std::chrono::steady_clock::time_point last_image_warning_time_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_buffer_warning_time_ =
    std::chrono::steady_clock::now();
};
}  // namespace hik_camera_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera_ros2_driver::HikCameraRos2DriverNode)
