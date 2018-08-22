#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/core/mat.hpp>
#include <scigl_render/scene/camera_intrinsics.hpp>
#include <scigl_render/scene/pose.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace scigl_render_ros
{
struct RosConvert
{
  /*!
  Convert a ROS transformation to a pose in the world coordinates of the
  rendering.
  */
  static scigl_render::QuaternionPose convert_tf(
      const geometry_msgs::TransformStamped &tf_pose);

  /*!
  Convert a ROS CameraInfo message to the renderings camera intrinsic
  parameters.
  */
  static scigl_render::CameraIntrinsics convert_camera_info(
      const sensor_msgs::CameraInfoConstPtr &camera_info,
      double min_depth = 0.1, double max_depth = 10);

  /*!
  Creates an OpenCV image from an ros image. The created image has the origin
  at the bottom left and is ready to use for OpenGL rendering.
  */
  static cv::Mat convert_ros_image(const sensor_msgs::ImageConstPtr &image,
                                   const std::string &encoding =
                                       sensor_msgs::image_encodings::BGR8);

  /*!
  Converts an image that was rendered by OpenGL to an image that can be 
  published in ROS. This means that the origin is in the top-left.
  */
  static sensor_msgs::ImagePtr convert_gl_image(
      cv::Mat image, const std::string &encoding =
                         sensor_msgs::image_encodings::BGR8);
};
} // namespace scigl_render_ros