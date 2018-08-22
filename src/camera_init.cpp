#include <future>
#include <image_transport/image_transport.h>
#include <scigl_render_ros/camera_init.hpp>

namespace scigl_render_ros
{
auto CameraInit::wait_for_info(const std::string &base_topic,
                               const ros::NodeHandle &nh)
    -> sensor_msgs::CameraInfoConstPtr
{
  // store the result in this future
  std::promise<sensor_msgs::CameraInfoConstPtr> info_promise;
  auto info_fut = info_promise.get_future();
  // subscribe the image
  image_transport::ImageTransport transport(nh);
  auto receiver = transport.subscribeCamera(
      base_topic, 1,
      [&info_promise](const sensor_msgs::ImageConstPtr &,
                      const sensor_msgs::CameraInfoConstPtr &info) {
        // set the promise so the future get() does not block anymore
        info_promise.set_value(info);
      });
  while (std::future_status::ready !=
         info_fut.wait_for(std::chrono::milliseconds(100)))
  {
    ros::spinOnce();
  }
  return info_fut.get();
}
} // namespace scigl_render_ros