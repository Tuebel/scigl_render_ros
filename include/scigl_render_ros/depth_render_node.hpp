#pragma once
#include <image_transport/image_transport.h>
#include <memory>
#include <ros/ros.h>
#include <scigl_render_ros/depth_render.hpp>
#include <tf2_ros/transform_listener.h>

namespace scigl_render_ros
{
/*!
Node that simulates depth values by rendering an object via the OpenGL.
Publishes as virtual camera and provides the intrinsic camera calibration.
*/
class DepthRenderNode
{
public:
  /*!
  Creates the node with parametrization from ros_param.
  */
  DepthRenderNode();

  /*!
  Runs the rendering, will block until shutdown;
  */
  void run();

private:
  ros::NodeHandle node_handle;
  double frame_rate;
  std::unique_ptr<DepthRender> depth_render;
  // receive tf2 messages
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  // publish images as virtual camera
  image_transport::ImageTransport img_transport;
  image_transport::CameraPublisher depth_publisher;
  // world frame is the base of the other frames
  std::string world_frame_id;
  std::string camera_frame_id;
  std::string object_frame_id;
  // model to render
  std::string model_path;
  // camera intrinsics
  scigl_render::CameraIntrinsics intrinsics;
  sensor_msgs::CameraInfoPtr cam_info;

  /*!
  Callback for rendering periodically
  */
  void timer_callback(const ros::TimerEvent& e);
};
} // namespace scigl_render_ros