#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);
  
  // Create a ROS2 node named ObiWan
  auto node = rclcpp::Node::make_shared("ObiWan");

  // We create a Rate object of 2Hz
  rclcpp::WallRate loop_rate(2);

  // Endless loop until Ctrl + C
  while (rclcpp::ok()) 
  {
    // Print a message to the terminal
    RCLCPP_INFO(node->get_logger(), "Help me Obi-Wan Kenobi, you're my only hope");
    rclcpp::spin_some(node);

    // We sleep the needed time to maintain the Rate fixed above
    loop_rate.sleep();
  }

  // Shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}
// This program creates an endless loop that repeats itself 2 times per second (2Hz) until somebody presses Ctrl + C
// in the Shell