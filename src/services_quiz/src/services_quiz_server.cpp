#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>

#include <memory>

using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode() : Node("rotate_server")
  {
    srv_ = create_service<Spin>("rotate", std::bind(&ServerNode::rotate_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void rotate_callback(
      const std::shared_ptr<Spin::Request> request,
      const std::shared_ptr<Spin::Response> response) 
    {
        auto message = geometry_msgs::msg::Twist();
        if (request->direction == "right")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right rotation...");
            message.angular.z = -1 * request->angular_velocity;
            publisher_->publish(message);
            std::this_thread::sleep_for(std::chrono::seconds{request->time});
            message.angular.z = 0.0;
            publisher_->publish(message);
            response->success = true;
        }
        else if (request->direction == "left")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left rotation...");
            message.angular.z = request->angular_velocity;
            publisher_->publish(message);
            std::this_thread::sleep_for(std::chrono::seconds{request->time});
            message.angular.z = 0.0;
            publisher_->publish(message);
            response->success = true;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid rotation...");
            response->success = false;
        }      
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}