#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ros/console.h>
#include <scigl_render/check_gl_error.hpp>
#include <scigl_render/shader/depth_shader.hpp>
#include <scigl_render_ros/depth_render.hpp>
#include <scigl_render_ros/scigl_convert.hpp>

namespace scigl_render_ros
{
// image format in OpenGL
const GLint INTERNAL_FORMAT = GL_R32F;
const GLenum FORMAT = GL_RED;
const GLenum TYPE = GL_FLOAT;
// matching format in OpenCV
const std::string IMAGE_ENCODING = sensor_msgs::image_encodings::TYPE_32FC1;

DepthRender::DepthRender(const std::string &model_path,
                         const scigl_render::CameraIntrinsics &intrinsics)
    : gl_context(false, false, intrinsics.width, intrinsics.height),
      offscreen_render(intrinsics.width, intrinsics.height, sizeof(float),
                       INTERNAL_FORMAT),
      camera(intrinsics),
      model(model_path),
      shader(scigl_render::DepthShader::create_shader())
{
  // cv::Mat::create will allocate continous memory
  image_buffer.create(intrinsics.height, intrinsics.width);
  // Configure the global rendering settings
  glColorMask(GL_TRUE, GL_FALSE, GL_FALSE, GL_FALSE);
  glEnable(GL_DEPTH_TEST);
  scigl_render::check_gl_error("creating depth render");
}

auto DepthRender::render(const geometry_msgs::TransformStamped &camera_pose,
                         const geometry_msgs::TransformStamped &object_pose)
    -> sensor_msgs::ImagePtr
{
  try
  {
    // convert the poses
    camera.pose = SciglConvert::convert_tf(camera_pose);
    model.pose = SciglConvert::convert_tf(object_pose);
    // render the object on top
    offscreen_render.clear_fbo();
    offscreen_render.activate_fbo();
    shader.activate();
    camera.set_in_shader(shader);
    model.draw(shader);
    offscreen_render.deactivate_fbo();
    // start reading this frame
    offscreen_render.start_read(FORMAT, TYPE);
    // read last frame into the buffer
    offscreen_render.read_data([this](const void *data) {
      // buffer has continous memory (use of create)
      memcpy(image_buffer.data, data,
             image_buffer.total() * image_buffer.elemSize());
    });
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  // create ros message
  auto res = SciglConvert::convert_gl_image(image_buffer, IMAGE_ENCODING);
  return res;
}
} // namespace scigl_render_ros