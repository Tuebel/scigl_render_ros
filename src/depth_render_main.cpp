#include <scigl_render_ros/depth_render_node.hpp>

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