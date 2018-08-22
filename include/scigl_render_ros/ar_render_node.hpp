#pragma once
#include <image_transport/image_transport.h>
#include <memory>
#include <ros/ros.h>
#include <scigl_render_ros/ar_render.hpp>
#include <tf2_ros/transform_listener.h>

namespace scigl_render_ros
{
/*!
Augmeneted reality rendering of 3D models via the CameraInfo topic and a given
transformation.
*/
class ArRenderNode
{
public:
  /*!
  Creates the node with parametrization from ros_param.
  */
  ArRenderNode();

  /*!
  Runs the rendering, will block until shutdown;
  */
  void run();

private:
  ros::NodeHandle node_handle;
  std::unique_ptr<ArRender> ar_render;
  // receive tf2 messages
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  // receive and publish images
  image_transport::ImageTransport img_transport;
  image_transport::CameraPublisher ar_publisher;
  // world frame is the base of the other frames
  std::string world_frame_id;
  std::string object_frame_id;
  std::string light_frame_id;
  // model to render
  std::string model_path;

  /*!
  Renders into the received image and publishes the AR image.
  */
  void camera_callback(const sensor_msgs::ImageConstPtr &img,
                       const sensor_msgs::CameraInfoConstPtr &info);
};
} // namespace scigl_render_ros