#include <scigl_render_ros/depth_render_node.hpp>
#include <scigl_render_ros/scigl_convert.hpp>

namespace scigl_render_ros
{

DepthRenderNode::DepthRenderNode() : tf_listener(tf_buffer),
                                     img_transport(node_handle)
{
  ros::NodeHandle private_nh("~");
  // publish at this frame rate
  private_nh.param<double>("frame_rate", frame_rate, 30);
  // frames
  private_nh.param<std::string>("world_frame_id", world_frame_id, "world");
  private_nh.param<std::string>("camera_frame_id", camera_frame_id, "camera");
  private_nh.param<std::string>("object_frame_id", object_frame_id, "object");
  // publisher for the augmented reality image
  depth_publisher = img_transport.advertiseCamera("/depth/image", 1);
  // model to render
  private_nh.param<std::string>("model_path", model_path,
                                "/model/default.3ds");
  // camera parameters (default kinect from
  // https://www.mrpt.org/tutorials/programming/miscellaneous/kinect-calibration/)
  private_nh.param<int>("width", intrinsics.width, 640);
  private_nh.param<int>("height", intrinsics.height, 480);
  private_nh.param<float>("near", intrinsics.near, 0.01);
  private_nh.param<float>("far", intrinsics.far, 10);
  private_nh.param<float>("c_x", intrinsics.c_x, 314.649173);
  private_nh.param<float>("c_y", intrinsics.c_y, 240.160459);
  private_nh.param<float>("f_x", intrinsics.f_x, 572.882768);
  private_nh.param<float>("f_y", intrinsics.f_y, 542.739980);
  private_nh.param<float>("s", intrinsics.s, 0);
  // quite a pain to create
  cam_info = SciglConvert::convert_intrinsics(intrinsics);
  cam_info->header.frame_id = camera_frame_id;
}

void DepthRenderNode::run()
{
  namespace ph = std::placeholders;
  depth_render = std::unique_ptr<DepthRender>(new DepthRender(model_path,
                                                              intrinsics));
  // publish images epriodically
  auto cam_timer = node_handle.createTimer(
      ros::Duration(1.0 / frame_rate),
      std::bind(&DepthRenderNode::timer_callback, this, ph::_1));
  // block to keep ar_render in scope
  ros::spin();
}

void DepthRenderNode::timer_callback(const ros::TimerEvent &)
{
  // Render with latest transform
  auto camera_pose = tf_buffer.lookupTransform(world_frame_id, camera_frame_id,
                                               ros::Time(0), ros::Duration(1));
  auto object_pose = tf_buffer.lookupTransform(world_frame_id, object_frame_id,
                                               ros::Time(0), ros::Duration(1));
  auto image = depth_render->render(camera_pose, object_pose);
  // fill in the headers
  image->header.stamp = ros::Time::now();
  image->header.frame_id = camera_frame_id;
  cam_info->header.stamp = image->header.stamp;
  cam_info->header.frame_id = image->header.frame_id;
  // publish it
  depth_publisher.publish(image, cam_info);
}
} // namespace scigl_render_ros

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "depth_render_node");
  ros::NodeHandle nh;
  scigl_render_ros::DepthRenderNode render_node;
  // will block/spin
  render_node.run();
  return EXIT_SUCCESS;
}
