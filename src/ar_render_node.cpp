#include <memory>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <scigl_render_ros/ar_render.hpp>
#include <tf2_ros/transform_listener.h>

class ArRenderNode
{
public:
  /*!
  Creates the node with parametrization from ros_param.
  */
  ArRenderNode() : tf_listener(tf_buffer),
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

  /*!
  Runs the rendering, will block until shutdown;
  */
  void run()
  {
    using namespace sensor_msgs;
    scigl_render_ros::ArRender ar_render(model_path, init_camera());
    // subscribe the images and publish the modified ones
    image_transport::CameraSubscriber ar_subscriber =
        img_transport.subscribeCamera(
            camera_base_topic, 1,
            [this, &ar_render](const ImageConstPtr &img, const CameraInfoConstPtr &info) {
              // Poses in target frame world
              try
              {
                auto camera_pose =
                    tf_buffer.lookupTransform(world_frame_id,
                                              info->header.frame_id,
                                              ros::Time(0));
                auto object_pose =
                    tf_buffer.lookupTransform(world_frame_id,
                                              object_frame_id, ros::Time(0));
                auto light_pose =
                    tf_buffer.lookupTransform(world_frame_id,
                                              light_frame_id, ros::Time(0));
                // Render the new image and publish it
                auto ar_image = ar_render.render(camera_pose, object_pose,
                                                 light_pose, img);
                ar_publisher.publish(ar_image, info);
              }
              catch (tf2::TransformException &ex)
              {
                ROS_WARN("%s", ex.what());
              }
            });
    // block to keep ar_render in scope
    ros::spin();
  }

private:
  ros::NodeHandle node_handle;
  // receive tf2 messages
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  // receive and publish images
  std::string camera_base_topic;
  image_transport::ImageTransport img_transport;
  image_transport::CameraPublisher ar_publisher;
  // world frame is the base of the other frames
  std::string world_frame_id;
  std::string object_frame_id;
  std::string light_frame_id;
  // model to render
  std::string model_path;
  /*!
  Waits for a camera message and extracts the info.
  */
  sensor_msgs::CameraInfoConstPtr init_camera()
  {
    auto parent_ns = ros::names::parentNamespace(camera_base_topic);
    auto info_topic = ros::names::append(parent_ns, "camera_info");
    return ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info_topic);
  }
};

/*!
Configures the object tracker, receives and sends the ros messages. This is the
interface to other ROS nodes.
*/
int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "ar_render_node");
  ros::NodeHandle nh;
  ArRenderNode render_node;
  // will block/spin
  render_node.run();
  return EXIT_SUCCESS;
}
