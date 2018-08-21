#pragma once
#include <opencv2/core/core.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gl3w/GL/gl3w.h>
#include <scigl_render/gl_context.hpp>
#include <scigl_render/render/offscreen_render.hpp>
#include <scigl_render/scene/cv_camera.hpp>
#include <scigl_render/scene/diffuse_light.hpp>
#include <scigl_render/scene/model.hpp>
#include <scigl_render/shader/shader.hpp>
#include <scigl_render/render/texture_fullscreen_render.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace scigl_render_ros
{
/*!
Augmented reality rendering of an object using the scigl_render library and
ROS tf2 and ROS images.
This class 
*/
class ArRender
{
public:
  /*!
  Configures the render class.
  \param model_path the 3D model to render
  \param camera_info contains the intrinsic parameters
  \param min_depth the shortest range to measure by the camera
  \param max_depth the longest range mo measure by the camera
  */
  ArRender(const std::string &model_path,
           const sensor_msgs::CameraInfoConstPtr &camera_info,
           double min_depth = 0.01, double max_depth = 10);

  /*!
  Renders the configured model using the cameras intrinsics. The model is
  rendered onto the image.
  \param camera_pose the pose of the camera n the world frame (ROS convention: 
  http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
  \param object_pose the pose of the object in the world frame
  \param light_pose the pose of the light in the world frame
  \param image the image message.
  \return an image with the object render into
  */
  sensor_msgs::ImageConstPtr render(
      const geometry_msgs::TransformStamped &camera_pose,
      const geometry_msgs::TransformStamped &object_pose,
      const geometry_msgs::TransformStamped &light_pose,
      const sensor_msgs::ImageConstPtr &image);

  /*!
  Creates opengl camera intrinsics from the camera info.
  */
  static scigl_render::CameraIntrinsics convert_camera_info(
      const sensor_msgs::CameraInfoConstPtr &camera_info,
      double min_depth = 0.2, double max_depth = 10);

private:
  // draw offscreen
  scigl_render::GLContext gl_context;
  scigl_render::OffscreenRender offscreen_render;
  cv::Mat image_buffer;
  uint32_t seq = 0;
  // rendering scene
  scigl_render::CvCamera camera;
  scigl_render::DiffuseLight light;
  scigl_render::Model model;
  // shaders
  scigl_render::Shader shader;
  scigl_render::TextureFullscreenRender image_render;

  /*!
  Converts a tf2 transformation to a pose that can be used in OpenGL.
  */
  static auto convert_pose(const geometry_msgs::TransformStamped &tf_pose)
      -> scigl_render::QuaternionPose;
};
} // namespace scigl_render_ros