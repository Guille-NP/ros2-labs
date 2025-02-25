#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <unistd.h>


using namespace std::chrono_literals;

class BoxBotManager : public rclcpp::Node {
public:
  BoxBotManager(std::string odom_topic_name_1, std::string odom_topic_name_2, std::string odom_topic_name_3, 
                geometry_msgs::msg::Point goal1, geometry_msgs::msg::Point goal2, geometry_msgs::msg::Point goal3)
                 : Node("box_bot_manager") 
  {
    this->goal1_ = goal1;
    this->goal2_ = goal2;
    this->goal3_ = goal3;

    callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_timer = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    rclcpp::SubscriptionOptions options2;
    rclcpp::SubscriptionOptions options3;

    options1.callback_group = callback_group_1;
    options2.callback_group = callback_group_2;
    options3.callback_group = callback_group_3;

    subscription_1 = this->create_subscription<nav_msgs::msg::Odometry>(
                     odom_topic_name_1, 10, 
                     std::bind(&BoxBotManager::topic_callback_1, this, std::placeholders::_1),
                     options1);
    subscription_2 = this->create_subscription<nav_msgs::msg::Odometry>(
                     odom_topic_name_2, 10, 
                     std::bind(&BoxBotManager::topic_callback_2, this, std::placeholders::_1),
                     options2);
    subscription_3 = this->create_subscription<nav_msgs::msg::Odometry>(
                     odom_topic_name_3, 10, 
                     std::bind(&BoxBotManager::topic_callback_3, this, std::placeholders::_1),
                     options3);

    this->wait_time = 1.0;
    timer_ = this->create_wall_timer(500ms, std::bind(&BoxBotManager::timer_callback, this), 
                                     callback_group_timer);

    publisher_boxbot_1 = this->create_publisher<std_msgs::msg::String>("/reached_goal/box_bot_1", 1);
    publisher_boxbot_2 = this->create_publisher<std_msgs::msg::String>("/reached_goal/box_bot_2", 1);
    publisher_boxbot_3 = this->create_publisher<std_msgs::msg::String>("/reached_goal/box_bot_3", 1);

  }

private:
  void topic_callback_1(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    this->boxbot_1_reached_goal = check_goal(goal1_, msg->pose.pose.position, "boxbot_1", 0.1);
  }

  void topic_callback_2(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    this->boxbot_2_reached_goal = check_goal(goal2_, msg->pose.pose.position, "boxbot_2", 0.1);
  }

  void topic_callback_3(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    this->boxbot_3_reached_goal = check_goal(goal3_, msg->pose.pose.position, "boxbot_3", 0.1);
  }

  bool check_goal(geometry_msgs::msg::Point goal, geometry_msgs::msg::Point position, std::string bot_name, float error = 0.1)
  {
    bool result = false;
    float curr_error_x = goal.x - position.x;
    float curr_error_y = goal.y - position.y;
    if (curr_error_x <= error && curr_error_x >= -1*error)
    {
        if (curr_error_y <= error && curr_error_y >= -1*error)
        {
            result = true;
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "'%s Y pos. error --> Goal.y: '%f', Positioned.y: '%f", bot_name.c_str(), goal.y, position.y);
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "'%s X pos. error --> Goal.x: '%f', Positioned.x: '%f", bot_name.c_str(), goal.x, position.x);
    }

    return result;
  }

  void timer_callback() 
  {
    if (this->boxbot_1_reached_goal)
    {
        std_msgs::msg::String message;
        message.data = "BOX BOT 1 REACHED GOAL =" + std::to_string(this->goal1_.x) + "," + std::to_string(this->goal1_.y);
        publisher_boxbot_1->publish(message);
        RCLCPP_INFO(this->get_logger(), "BOT 1 REACHED GOAL!!");
    }
    if (this->boxbot_2_reached_goal)
    {
        std_msgs::msg::String message;
        message.data = "BOX BOT 2 REACHED GOAL =" + std::to_string(this->goal2_.x) + "," + std::to_string(this->goal2_.y);
        publisher_boxbot_2->publish(message);
        RCLCPP_INFO(this->get_logger(), "BOT 2 REACHED GOAL!!");
    }
    if (this->boxbot_3_reached_goal)
    {
        std_msgs::msg::String message;
        message.data = "BOX BOT 3 REACHED GOAL =" + std::to_string(this->goal3_.x) + "," + std::to_string(this->goal3_.y);
        publisher_boxbot_3->publish(message);
        RCLCPP_INFO(this->get_logger(), "BOT 3 REACHED GOAL!!");
    }
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;
  rclcpp::CallbackGroup::SharedPtr callback_group_timer;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_1;  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_2;  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_3;  

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_boxbot_1;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_boxbot_2;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_boxbot_3;

  rclcpp::TimerBase::SharedPtr timer_;
  float wait_time;

  geometry_msgs::msg::Point goal1_;
  geometry_msgs::msg::Point goal2_;
  geometry_msgs::msg::Point goal3_;

  bool boxbot_1_reached_goal = false;
  bool boxbot_2_reached_goal = false;
  bool boxbot_3_reached_goal = false;
};


int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  geometry_msgs::msg::Point goal1;
  geometry_msgs::msg::Point goal2;
  geometry_msgs::msg::Point goal3;

  goal1.x = 2.498409;        //Original: 2.498409   Initial: 0.498409
  goal1.y = -1.132045;       //Original: -1.132045  Initial: -1.497
  std::string name1 = "/box_bot_1/odom";
  goal2.x = 0.974281;        //Original: 0.974281   Initial: 1.099
  goal2.y = -1.132045;       //Original: -1.132045  Initial: -1.489
  std::string name2 = "/box_bot_2/odom";
  goal3.x = -0.507990;       //Original: -0.507990  Initial: 1.699
  goal3.y = -1.132045;       //Original: -1.132045  Initial:-1.489 
  std::string name3 = "/box_bot_3/odom";

  // Instantiate the Node
  std::shared_ptr<BoxBotManager> boxbot_manager = std::make_shared<BoxBotManager>(name1, name2, name3, goal1, goal2, goal3);

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(boxbot_manager);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}