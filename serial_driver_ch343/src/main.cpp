#include <rclcpp/rclcpp.hpp>
#include "serial_driver_ch343/ReadNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<serial_driver::ReadNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}