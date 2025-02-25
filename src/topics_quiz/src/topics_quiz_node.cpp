#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TopicsQuiz : public rclcpp::Node
{
public:
  TopicsQuiz()
  : Node("topics_quiz_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "scan", 10, std::bind(&TopicsQuiz::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto message = geometry_msgs::msg::Twist();
    //RCLCPP_INFO(this->get_logger(), "HERE");
    message.linear.x = 0.5;
    message.angular.z = 0.0;

    //for (int i=360; i>=0; --i)
    for (int i=360; i<=719; ++i)
    {
        if (msg->ranges[i] < 1.0)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.5;
            break;
        }
    }

    /*if (msg->ranges[360] < 1.0 || msg->ranges[330] < 1.0)
    {
        message.linear.x = 0.0;
        message.angular.z = 0.5;
    }
    else if (msg->ranges[0] < 1.0)
    {
        message.linear.x = 0.0;
        message.angular.z = 0.5;
    }
    else
    {
        message.linear.x = 0.3;
        message.angular.z = 0.0;
    }*/

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicsQuiz>());
  rclcpp::shutdown();
  return 0;
}