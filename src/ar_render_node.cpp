#include <future>
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
    private_nh.param<std::string>("camera_frame_id", camera_frame_id, "camera");
    private_nh.param<std::string>("object_frame_id", object_frame_id, "object");
    // camera topic
    private_nh.param<std::string>(
        "camera_base_topic", camera_base_topic, "/camera");
    // publisher for the augmented reality image
    ar_publisher = img_transport.advertiseCamera("/ar_camera", 1);
    // model to render
    private_nh.param<std::string>("model_path", model_path);
  }

  /*!
  Runs the rendering, will block until shutdown;
  */
  void run()
  {
    using namespace sensor_msgs;
    scigl_render_ros::ArRender ar_render(model_path, init_camera());
    // subscribe the images and publish the modified ones
    image_transport::CameraSubscriber subscriber =
        img_transport.subscribeCamera(
            camera_base_topic, 1,
            [this, &ar_render](const ImageConstPtr &image, CameraInfoConstPtr info) {
              // Get the latest poses
              auto camera_pose =
                  tf_buffer.lookupTransform(
                      camera_frame_id, world_frame_id, ros::Time(0));
              auto object_pose =
                  tf_buffer.lookupTransform(
                      object_frame_id, world_frame_id, ros::Time(0));
              // Render the new image and publish it
              auto ar_image = ar_render.render(camera_pose, object_pose, image);
              ar_publisher.publish(ar_image, info);
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
  std::string camera_frame_id;
  std::string object_frame_id;
  // model to render
  std::string model_path;

  /*!
  Waits for a camera message and extracts the info.
  */
  sensor_msgs::CameraInfoConstPtr init_camera()
  {
    using namespace image_transport;
    using namespace sensor_msgs;
    // create a promise that will be filled when receiving the first image
    std::promise<CameraInfoConstPtr> info_promise;
    CameraSubscriber subscriber =
        img_transport.subscribeCamera(
            camera_base_topic, 1,
            [&info_promise](const ImageConstPtr &, CameraInfoConstPtr info) {
              info_promise.set_value(info);
            });
    // Get the value from the promise
    return info_promise.get_future().get();
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
  ArRenderNode node;
  // will block/spin
  node.run();
  return EXIT_SUCCESS;
}