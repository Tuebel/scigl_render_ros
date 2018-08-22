#include <scigl_render_ros/scigl_convert.hpp>
#include <cv_bridge/cv_bridge.h>

namespace scigl_render_ros
{
scigl_render::QuaternionPose SciglConvert::convert_tf(
    const geometry_msgs::TransformStamped &tf_pose)
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

scigl_render::CameraIntrinsics SciglConvert::convert_camera_info(
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

sensor_msgs::CameraInfoPtr SciglConvert::convert_intrinsics(
    const scigl_render::CameraIntrinsics &intrinsics)
{
  sensor_msgs::CameraInfoPtr info;
  //dimensions
  info->width = intrinsics.width;
  info->height = intrinsics.height;
  // matrices
  info->K = {intrinsics.f_x, 0, intrinsics.c_x,
             0, intrinsics.f_y, intrinsics.c_y,
             0, 0, 1};
  info->P = {intrinsics.f_x, 0, intrinsics.c_x, 0,
             0, intrinsics.f_y, intrinsics.c_y, 0,
             0, 0, 1, 0};
  return info;
}

cv::Mat SciglConvert::convert_ros_image(const sensor_msgs::ImageConstPtr &image,
                                        const std::string &encoding)
{
  auto cv_ptr = cv_bridge::toCvShare(image, encoding);
  auto gl_image = cv_ptr->image;
  cv::flip(gl_image, gl_image, 0);
  return gl_image;
}

sensor_msgs::ImagePtr SciglConvert::convert_gl_image(
    cv::Mat image, const std::string &encoding)
{
  cv::flip(image, image, 0);
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  cv_bridge::CvImage cv_image(header, encoding, image);
  return cv_image.toImageMsg();
}
} // namespace scigl_render_ros
