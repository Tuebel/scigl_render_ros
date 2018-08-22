#include <scigl_render_ros/ar_render_node.hpp>

namespace scigl_render_ros
{

ArRenderNode::ArRenderNode() : tf_listener(tf_buffer),
                               img_transport(node_handle)
{
  ros::NodeHandle private_nh("~");
  // frames
  private_nh.param<std::string>("world_frame_id", world_frame_id, "world");
  private_nh.param<std::string>("object_frame_id", object_frame_id, "object");
  private_nh.param<std::string>("light_frame_id", light_frame_id, "light");
  // camera topic
  private_nh.param<std::string>(
      "camera_base_topic", camera_base_topic, "/camera/image_raw");
  // publisher for the augmented reality image
  ar_publisher = img_transport.advertiseCamera("/ar_image/image_raw", 1);
  // model to render
  private_nh.param<std::string>("model_path", model_path,
                                "/model/default.3ds");
}

void ArRenderNode::run()
{
  namespace ph = std::placeholders;
  ar_render = std::unique_ptr<ArRender>(new ArRender(
      model_path, init_camera()));
  // subscribe the images and publish the modified ones
  image_transport::CameraSubscriber ar_subscriber =
      img_transport.subscribeCamera(camera_base_topic, 1,
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
    ar_publisher.publish(ar_image, info);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

sensor_msgs::CameraInfoConstPtr ArRenderNode::init_camera()
{
  auto parent_ns = ros::names::parentNamespace(camera_base_topic);
  auto info_topic = ros::names::append(parent_ns, "camera_info");
  return ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info_topic);
}
} // namespace scigl_render_ros
