#include <scigl_render_ros/ar_render_node.hpp>
#include <scigl_render_ros/camera_init.hpp>

namespace scigl_render_ros
{
const std::string CAMERA_TOPIC = "camera/image_raw";

ArRenderNode::ArRenderNode() : tf_listener(tf_buffer),
                               img_transport(node_handle)
{
  ros::NodeHandle private_nh("~");
  // frames
  private_nh.param<std::string>("world_frame_id", world_frame_id, "world");
  private_nh.param<std::string>("object_frame_id", object_frame_id, "object");
  private_nh.param<std::string>("light_frame_id", light_frame_id, "light");
  // publisher for the augmented reality image
  ar_publisher = img_transport.advertiseCamera("/ar_image/image_raw", 1);
  // model to render
  private_nh.param<std::string>("model_path", model_path, "/model/default.3ds");
}

void ArRenderNode::run()
{
  namespace ph = std::placeholders;
  auto camera_info = CameraInit::wait_for_info(CAMERA_TOPIC, node_handle);
  ar_render = std::unique_ptr<ArRender>(new ArRender(
      model_path, camera_info));
  ROS_ERROR("ready to render");
  // subscribe the images and publish the modified ones
  image_transport::CameraSubscriber ar_subscriber =
      img_transport.subscribeCamera(CAMERA_TOPIC, 1,
                                    std::bind(&ArRenderNode::camera_callback,
                                              this, ph::_1, ph::_2));
  // block to keep ar_render in scope
  ros::spin();
}

void ArRenderNode::camera_callback(const sensor_msgs::ImageConstPtr &img,
                                   const sensor_msgs::CameraInfoConstPtr &info)
{
  try
  {
    auto camera_pose = tf_buffer.lookupTransform(world_frame_id,
                                                 info->header.frame_id,
                                                 ros::Time(0));
    auto object_pose = tf_buffer.lookupTransform(world_frame_id,
                                                 object_frame_id, ros::Time(0));
    auto light_pose = tf_buffer.lookupTransform(world_frame_id,
                                                light_frame_id, ros::Time(0));
    // Render the new image and publish it
    auto ar_image = ar_render->render(camera_pose, object_pose,
                                      light_pose, img);
    ar_image->header.stamp = ros::Time::now();
    // create copy to update time.
    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(*info));
    cam_info->header.stamp = ros::Time::now();
    ar_publisher.publish(ar_image, cam_info);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}
} // namespace scigl_render_ros
