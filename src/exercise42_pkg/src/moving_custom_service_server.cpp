#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

class CustomServerNode : public rclcpp::Node
{
public:
  CustomServerNode() : Node("service_moving_custom")
  {
    srv_ = create_service<std_srvs::srv::SetBool>("custom_moving", 
                          std::bind(&CustomServerNode::moving_custom_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void moving_custom_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        auto message = geometry_msgs::msg::Twist();
        if (request->data == true)
        {
            message.linear.x = 0.2;
            message.angular.z = 0.2;
            publisher_->publish(message);
            response->success = true;
        }
        else if (request->data == false)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomServerNode>());
  rclcpp::shutdown();
  return 0;
}