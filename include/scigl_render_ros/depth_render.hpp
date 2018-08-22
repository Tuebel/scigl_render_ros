#pragma once
#include <opencv2/core/core.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <gl3w/GL/gl3w.h>
#include <scigl_render/gl_context.hpp>
#include <scigl_render/render/offscreen_render.hpp>
#include <scigl_render/scene/cv_camera.hpp>
#include <scigl_render/scene/model.hpp>
#include <scigl_render/shader/shader.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace scigl_render_ros
{
/*!
Simulating depth values using the scigl_render library
*/
class DepthRender
{
public:
  /*!
  Configures the render class.
  \param model_path the 3D model to render
  \param camera_info contains the intrinsic parameters
  \param min_depth the shortest range to measure by the camera
  \param max_depth the longest range mo measure by the camera
  */
  DepthRender(const std::string &model_path,
              const scigl_render::CameraIntrinsics &intrinsics);

  /*!
  Renders the configured model as depth image using the cameras intrinsics.
  \param camera_pose the pose of the camera n the world frame (ROS convention: 
  http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
  \param object_pose the pose of the object in the world frame
  \return an image with the object render into
  */
  sensor_msgs::ImagePtr render(
      const geometry_msgs::TransformStamped &camera_pose,
      const geometry_msgs::TransformStamped &object_pose);

private:
  // draw offscreen
  scigl_render::GLContext gl_context;
  scigl_render::OffscreenRender offscreen_render;
  cv::Mat_<float> image_buffer;
  // rendering scene
  scigl_render::CvCamera camera;
  scigl_render::Model model;
  // shaders
  scigl_render::Shader shader;
};
} // namespace scigl_render_ros