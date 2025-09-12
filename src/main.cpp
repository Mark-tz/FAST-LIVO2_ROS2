#include "LIVMapper.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  // replaces ros::init
  auto node = std::make_shared<LIVMapper>("laserMapping");  // LIVMapper now inherits from rclcpp::Node
  rclcpp::spin(node);  // replaces mapper.run() and ros::spin
  rclcpp::shutdown();
  return 0;
}