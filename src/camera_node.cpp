#include "camera_node.h"

namespace hikrobot_ros {

CameraNode::CameraNode(const rclcpp::NodeOptions &options)
    : Node("hikrobot_driver_node", options), frame_rate_hz_(100) {
  RCLCPP_INFO(this->get_logger(), "Starting CameraNode!");

  MV_CC_DEVICE_INFO_LIST DeviceList;
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &DeviceList);
  RCLCPP_INFO(this->get_logger(), "Found camera count = %d",
              DeviceList.nDeviceNum);

  while (DeviceList.nDeviceNum == 0 && rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &DeviceList);
  }

  nRet = MV_CC_CreateHandle(&camera_handle_, DeviceList.pDeviceInfo[0]);
  if (nRet != MV_OK) {
    RCLCPP_INFO(this->get_logger(),
                "MV_CC_CreateHandle fail :: error_code [%x]", nRet);
  }

  nRet = MV_CC_OpenDevice(camera_handle_);
  if (nRet != MV_OK) {
    RCLCPP_INFO(this->get_logger(), "MV_CC_OpenDevice :: error_code [%x]",
                nRet);
  }

  if (DeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE) {
    int int_PacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);

    if (int_PacketSize > 0) {
      nRet = MV_CC_SetIntValue(camera_handle_, "GevSCPSPacketSize",
                               int_PacketSize);
      if (nRet != MV_OK) {
        RCLCPP_INFO(this->get_logger(),
                    "Warning: Set Packet Size fail :: error_code [%x]", nRet);
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Warning: Get Packet Size fail :: error_code [%x]",
                  int_PacketSize);
    }
  }

  bool use_sensor_data_qos =
      this->declare_parameter("use_sensor_data_qos", false);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                 : rmw_qos_profile_default;
  camera_pub_ =
      image_transport::create_camera_publisher(this, "image_raw", qos);

  declareParameters();

  MV_CC_GetImageInfo(camera_handle_, &img_info_);
  image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

  ConvertParam_.nWidth = img_info_.nWidthMax;
  ConvertParam_.nHeight = img_info_.nHeightMax;
  ConvertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

  MV_CC_StartGrabbing(camera_handle_);

  camera_name_ = this->declare_parameter("camera_name", "hikrobot");
  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this,
                                                               camera_name_);
  auto camera_info_url = this->declare_parameter(
      "camera_info_url",
      "package://hikrobot_ros2_driver/config/camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url)) {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                camera_info_url.c_str());
  }

  params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CameraNode::parametersCallback, this, std::placeholders::_1));

  capture_thread_ = std::thread{[this]() -> void {
    MV_FRAME_OUT OutFrame;

    RCLCPP_INFO(this->get_logger(), "Publishing image!");

    image_msg_.header.frame_id = "camera_optical_frame";
    image_msg_.encoding = "rgb8";

    rclcpp::WallRate loop_rate(frame_rate_hz_);
    while (rclcpp::ok()) {
      this->captureAndPublish(OutFrame);

      loop_rate.sleep();
    }
  }};
}

CameraNode::~CameraNode() {
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (camera_handle_) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
  }
  RCLCPP_INFO(this->get_logger(), "CameraNode destroyed!");
}

void CameraNode::captureAndPublish(MV_FRAME_OUT &OutFrame) {
  nRet = MV_CC_GetImageBuffer(camera_handle_, &OutFrame, 1000);
  if (MV_OK == nRet) {
    ConvertParam_.pDstBuffer = image_msg_.data.data();
    ConvertParam_.nDstBufferSize = image_msg_.data.size();
    ConvertParam_.pSrcData = OutFrame.pBufAddr;
    ConvertParam_.nSrcDataLen = OutFrame.stFrameInfo.nFrameLen;
    ConvertParam_.enSrcPixelType = OutFrame.stFrameInfo.enPixelType;

    MV_CC_ConvertPixelType(camera_handle_, &ConvertParam_);

    camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
    image_msg_.height = OutFrame.stFrameInfo.nHeight;
    image_msg_.width = OutFrame.stFrameInfo.nWidth;
    image_msg_.step = OutFrame.stFrameInfo.nWidth * 3;
    image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);
    camera_pub_.publish(image_msg_, camera_info_msg_);

    MV_CC_FreeImageBuffer(camera_handle_, &OutFrame);
  } else {
    RCLCPP_INFO(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_StartGrabbing(camera_handle_);
  }
}

void CameraNode::declareParameters() {
  int width;
  this->get_parameter_or<int>("width", width, 640);
  nRet = MV_CC_SetIntValue(camera_handle_, "Width", width);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "Width", width);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "Width", width,
                nRet);
  }

  int height;
  this->get_parameter_or<int>("height", height, 480);
  nRet = MV_CC_SetIntValue(camera_handle_, "Height", height);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "Height", height);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "Height", height,
                nRet);
  }

  int offset_x;
  this->get_parameter_or<int>("offset_x", offset_x, 0);
  nRet = MV_CC_SetIntValue(camera_handle_, "OffsetX", offset_x);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "OffsetX",
                offset_x);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "OffsetX",
                offset_x, nRet);
  }

  int offset_y;
  this->get_parameter_or<int>("offset_y", offset_y, 0);
  nRet = MV_CC_SetIntValue(camera_handle_, "OffsetY", offset_y);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "OffsetY",
                offset_y);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "OffsetY",
                offset_y, nRet);
  }

  bool frame_rate_enable;
  this->get_parameter_or<bool>("frame_rate_enable", frame_rate_enable, false);
  nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable",
                            frame_rate_enable);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %s",
                "AcquisitionFrameRateEnable",
                frame_rate_enable ? "true" : "false");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %s :: error_code [%x]",
                "AcquisitionFrameRateEnable",
                frame_rate_enable ? "true" : "false", nRet);
  }

  if (frame_rate_enable) {
    double frame_rate;
    this->get_parameter_or<double>("frame_rate", frame_rate, 25);
    nRet =
        MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate);
    if (MV_OK == nRet) {
      RCLCPP_INFO(this->get_logger(), "%s has been set to %.1f",
                  "AcquisitionFrameRate", frame_rate);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "%s failed :: value = %.1f :: error_code [%x]",
                  "AcquisitionFrameRate", frame_rate, nRet);
    }

    frame_rate_hz_ = frame_rate;
  }

  int burst_frame_count;
  this->get_parameter_or<int>("burst_frame_count", burst_frame_count, 1);
  nRet = MV_CC_SetIntValue(camera_handle_, "AcquisitionBurstFrameCount",
                           burst_frame_count);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d",
                "AcquisitionBurstFrameCount", burst_frame_count);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]",
                "AcquisitionBurstFrameCount", burst_frame_count, nRet);
  }

  int exposure_auto;
  this->get_parameter_or<int>("exposure_auto", exposure_auto, 0);
  nRet = MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", exposure_auto);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "ExposureAuto",
                exposure_auto);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "ExposureAuto",
                exposure_auto, nRet);
  }

  double exposure_time;
  this->get_parameter_or<double>("exposure_time", exposure_time, 5000);
  nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %.1f", "ExposureTime",
                exposure_time);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %.1f :: error_code [%x]", "ExposureTime",
                exposure_time, nRet);
  }

  bool gamma_enable;
  this->get_parameter_or<bool>("gamma_enable", gamma_enable, false);
  nRet = MV_CC_SetBoolValue(camera_handle_, "GammaEnable", gamma_enable);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %s", "GammaEnable",
                gamma_enable ? "true" : "false");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %s :: error_code [%x]", "GammaEnable",
                gamma_enable ? "true" : "false", nRet);
  }

  if (gamma_enable) {
    double gamma;
    this->get_parameter_or<double>("gamma", gamma, 0.f);
    nRet = MV_CC_SetFloatValue(camera_handle_, "Gamma", gamma);
    if (MV_OK == nRet) {
      RCLCPP_INFO(this->get_logger(), "%s has been set to %.1f", "Gamma",
                  gamma);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "%s failed :: value = %.1f :: error_code [%x]", "Gamma",
                  gamma, nRet);
    }
  }

  int gain_auto;
  this->get_parameter_or<int>("gain_auto", gain_auto, 0);
  nRet = MV_CC_SetEnumValue(camera_handle_, "GainAuto", gain_auto);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "GainAuto",
                gain_auto);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "GainAuto",
                gain_auto, nRet);
  }

  bool saturation_enable;
  this->get_parameter_or<bool>("saturation_enable", saturation_enable, false);
  nRet =
      MV_CC_SetBoolValue(camera_handle_, "SaturationEnable", saturation_enable);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %s", "SaturationEnable",
                saturation_enable ? "true" : "false");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %s :: error_code [%x]",
                "SaturationEnable", saturation_enable ? "true" : "false", nRet);
  }

  if (saturation_enable) {
    int saturation;
    this->get_parameter_or<int>("saturation", saturation, 0);
    nRet = MV_CC_SetIntValue(camera_handle_, "Saturation", saturation);
    if (MV_OK == nRet) {
      RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "Saturation",
                  saturation);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "%s failed :: value = %d :: error_code [%x]", "Saturation",
                  saturation, nRet);
    }
  }

  int trigger_mode;
  this->get_parameter_or<int>("trigger_mode", trigger_mode, 0);
  nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", trigger_mode);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "TriggerMode",
                trigger_mode);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "TriggerMode",
                trigger_mode, nRet);
  }

  int trigger_source;
  this->get_parameter_or<int>("trigger_source", trigger_source, 0);
  nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerSource", trigger_source);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "TriggerSource",
                trigger_source);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "TriggerSource",
                trigger_source, nRet);
  }

  int line_selector;
  this->get_parameter_or<int>("line_selector", line_selector, 0);
  nRet = MV_CC_SetEnumValue(camera_handle_, "LineSelector", line_selector);
  if (MV_OK == nRet) {
    RCLCPP_INFO(this->get_logger(), "%s has been set to %d", "v",
                line_selector);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "%s failed :: value = %d :: error_code [%x]", "LineSelector",
                line_selector, nRet);
  }
}

rcl_interfaces::msg::SetParametersResult CameraNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
    if (param.get_name() == "exposure_time") {
      int status =
          MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
      if (MV_OK != status) {
        result.successful = false;
        result.reason =
            "Failed to set exposure time, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "gain") {
      int status =
          MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      if (MV_OK != status) {
        result.successful = false;
        result.reason =
            "Failed to set gain, status = " + std::to_string(status);
      }
    } else {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
    }
  }
  return result;
}

} // namespace hikrobot_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hikrobot_ros::CameraNode)