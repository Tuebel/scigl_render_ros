#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ros/console.h>
#include <scigl_render/check_gl_error.hpp>
#include <scigl_render/shader/single_texture_shader.hpp>
#include <scigl_render_ros/ar_render.hpp>

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
                       3 * 1, INTERNAL_FORMAT),
      camera(convert_camera_info(camera_info, min_depth, max_depth)),
      model(model_path),
      shader(scigl_render::SingleTextureShader::create_shader()),
      image_render(camera_info->width, camera_info->height, INTERNAL_FORMAT)
{
  // cv::Mat::create will allocate continous memory
  image_buffer.create(camera_info->height, camera_info->width, CV_TYPE);
  light.color = glm::vec3(1, 1, 1);
  // Configure OpenGL
  // Configure the global rendering settings
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glEnable(GL_DEPTH_TEST);
  scigl_render::check_gl_error("creatin ArRender");
}

auto ArRender::render(const geometry_msgs::TransformStamped &camera_pose,
                      const geometry_msgs::TransformStamped &object_pose,
                      const sensor_msgs::ImageConstPtr &image)
    -> sensor_msgs::ImageConstPtr
{
  try
  {
    // convert the image to origin bottom left
    auto cv_ptr = cv_bridge::toCvShare(image, IMAGE_ENCODING);
    auto gl_image = cv_ptr->image;
    cv::flip(cv_ptr->image, gl_image, 0);
    // convert the poses
    camera.pose = convert_pose(camera_pose);
    light.position = camera.pose.position;
    model.pose = convert_pose(object_pose);
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
    // flip the buffer for the origin in the top-left
    cv::flip(image_buffer, image_buffer, 0);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  // create ros message
  cv_bridge::CvImage cv_image(image->header, IMAGE_ENCODING, image_buffer);
  cv_image.header.seq = seq++;
  cv_image.header.stamp = ros::Time::now();
  scigl_render::check_gl_error("rendering");
  return cv_image.toImageMsg();
}

scigl_render::CameraIntrinsics ArRender::convert_camera_info(
    const sensor_msgs::CameraInfoConstPtr &camera_info,
    double min_depth, double max_depth)
{
  scigl_render::CameraIntrinsics camera_intrinsics;
  // dimension
  camera_intrinsics.width = camera_info->width;
  camera_intrinsics.height = camera_info->height;
  // intrinsic matrix K
  camera_intrinsics.f_x = camera_info->K[0];
  camera_intrinsics.f_y = camera_info->K[4];
  camera_intrinsics.c_x = camera_info->K[2];
  camera_intrinsics.c_y = camera_info->K[5];
  camera_intrinsics.s = 0;
  // depth range for OpenGL
  if (min_depth == 0)
  {
    // Otherwise the OpenGL z-Buffer will not work!
    camera_intrinsics.near = 0.1;
  }
  else
  {
    camera_intrinsics.near = min_depth;
  }
  camera_intrinsics.far = max_depth;
  return camera_intrinsics;
}

auto ArRender::convert_pose(const geometry_msgs::TransformStamped &tf_pose)
    -> scigl_render::QuaternionPose
{
  scigl_render::QuaternionPose result;
  result.position.x = tf_pose.transform.translation.x;
  result.position.y = tf_pose.transform.translation.y;
  result.position.z = tf_pose.transform.translation.z;
  result.orientation.w = tf_pose.transform.rotation.w;
  result.orientation.x = tf_pose.transform.rotation.x;
  result.orientation.y = tf_pose.transform.rotation.y;
  result.orientation.z = tf_pose.transform.rotation.z;
  return result;
}
} // namespace scigl_render_ros