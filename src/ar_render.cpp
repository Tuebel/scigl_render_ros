#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ros/console.h>
#include <scigl_render/check_gl_error.hpp>
#include <scigl_render/shader/single_texture_shader.hpp>
#include <scigl_render_ros/ar_render.hpp>
#include <scigl_render_ros/scigl_convert.hpp>

namespace scigl_render_ros
{
// image format in OpenGL
const GLint INTERNAL_FORMAT = GL_RGB8;
const GLenum FORMAT = GL_BGR;
const GLenum TYPE = GL_UNSIGNED_BYTE;
// matching format in OpenCV
const std::string IMAGE_ENCODING = sensor_msgs::image_encodings::BGR8;
const int CV_TYPE = CV_8UC3;

ArRender::ArRender(const std::string &model_path,
                   const sensor_msgs::CameraInfoConstPtr &camera_info,
                   double min_depth, double max_depth)
    : gl_context(false, false, camera_info->width, camera_info->height),
      offscreen_render(camera_info->width, camera_info->height,
                       3 * sizeof(unsigned char), INTERNAL_FORMAT),
      camera(SciglConvert::convert_camera_info(camera_info, min_depth,
                                               max_depth)),
      model(model_path),
      shader(scigl_render::SingleTextureShader::create_shader()),
      image_render(camera_info->width, camera_info->height, INTERNAL_FORMAT)
{
  // cv::Mat::create will allocate continous memory
  image_buffer.create(camera_info->height, camera_info->width, CV_TYPE);
  light.color = glm::vec3(1, 1, 1);
  // Configure the global rendering settings
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);
  glEnable(GL_DEPTH_TEST);
  scigl_render::check_gl_error("creating ArRender");
}

auto ArRender::render(const geometry_msgs::TransformStamped &camera_pose,
                      const geometry_msgs::TransformStamped &object_pose,
                      const geometry_msgs::TransformStamped &light_pose,
                      const sensor_msgs::ImageConstPtr &image)
    -> sensor_msgs::ImagePtr
{
  try
  {
    // convert the image format (bottom left origin)
    auto gl_image = SciglConvert::convert_ros_image(image, IMAGE_ENCODING);
    // convert the poses
    camera.pose = SciglConvert::convert_tf(camera_pose);
    light.position = SciglConvert::convert_tf(light_pose).position;
    model.pose = SciglConvert::convert_tf(object_pose);
    // render the image
    offscreen_render.clear_fbo();
    offscreen_render.activate_fbo();
    image_render.draw(gl_image.data, FORMAT, TYPE);
    // render the object on top
    shader.activate();
    camera.set_in_shader(shader);
    light.set_in_shader(shader);
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
  res->header.frame_id = image->header.frame_id;
  res->header.stamp = image->header.stamp;
  return res;
}
} // namespace scigl_render_ros