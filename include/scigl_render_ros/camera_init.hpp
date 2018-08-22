#pragma once
#include <sensor_msgs/CameraInfo.h>
#include <ros/node_handle.h>

namespace scigl_render_ros
{
struct CameraInit
{
  /*!
  Waits for the CamerInfo message using the image_transport camera subscriber.
  A little bit of c++ future magic and ROS spinning blocks unil the message is
  received.
  */
  static auto wait_for_info(const std::string &base_topic,
                            const ros::NodeHandle &nh)
      -> sensor_msgs::CameraInfoConstPtr;
};
} // namespace scigl_render_ros