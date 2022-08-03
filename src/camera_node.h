#ifndef HIKROBOT_MV_ROS_CAMERA_NODE_H
#define HIKROBOT_MV_ROS_CAMERA_NODE_H

#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hikrobot_ros {

class CameraNode : public rclcpp::Node {
public:
  explicit CameraNode(const rclcpp::NodeOptions &options);
  ~CameraNode();

private:
  void declareParameters();
  void captureAndPublish(MV_FRAME_OUT &OutFrame);

  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  int nRet = MV_OK;
  void *camera_handle_;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM ConvertParam_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::thread capture_thread_;

  double frame_rate_hz_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

} // namespace hikrobot_ros

#endif // HIKROBOT_MV_ROS_CAMERA_NODE_H
