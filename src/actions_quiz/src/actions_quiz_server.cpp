#include <functional>
#include <math.h>
#include <memory>
#include <thread>

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "actions_quiz_msg/action/distance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

using std::pow;

class MyActionServer : public rclcpp::Node {
public:
  using Odometry = nav_msgs::msg::Odometry;
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;

  explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("distance_as_node", options)
  {
    using namespace std::placeholders;

    callback_group_as = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_subs = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    //options_as.callback_group = callback_group_as;
    options_subs.callback_group = callback_group_subs;

    this->action_server_ = rclcpp_action::create_server<Distance>(
      this,
      "distance_as",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1),
      rcl_action_server_get_default_options(),
      callback_group_as);

    subscription_ = this->create_subscription<Odometry>(
      "odom", 10, 
      std::bind(&MyActionServer::odom_callback, this, _1), 
      options_subs);

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);

  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;  

  rclcpp::CallbackGroup::SharedPtr callback_group_as;
  rclcpp::CallbackGroup::SharedPtr callback_group_subs;
  //rclcpp::SubscriptionOptions options_as;
  rclcpp::SubscriptionOptions options_subs;

  double curr_pos_x;
  double curr_pos_y;
  double last_pos_x;
  double last_pos_y;
  std_msgs::msg::Float64 distance;
  bool first_odom_meas = false;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (this->first_odom_meas)
    {
        distance.data = 0.0;
        this->last_pos_x = msg->pose.pose.position.x;
        this->last_pos_y = msg->pose.pose.position.y;
        this->first_odom_meas = false;
        RCLCPP_INFO(this->get_logger(), "First odom meas %f", distance.data);
    }

    this->curr_pos_x = msg->pose.pose.position.x;
    this->curr_pos_y = msg->pose.pose.position.y;
    distance.data += sqrt(pow(this->curr_pos_x - this->last_pos_x, 2) + 
                             pow(this->curr_pos_y - this->last_pos_y, 2));
    this->last_pos_x = this->curr_pos_x;
    this->last_pos_y = this->curr_pos_y;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Distance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->seconds);
    (void)uuid;
    this->first_odom_meas = true;
    distance.data = 0.0;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto &message = feedback->current_dist;
    message = this->distance.data;
    auto result = std::make_shared<Distance::Result>();

    rclcpp::Rate loop_rate(1);

    if (this->first_odom_meas)
    {
        this->first_odom_meas = false;
    }

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) 
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) 
      {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Send feedback
      message = distance.data;
      publisher_->publish(distance);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "current_dist: %lf", message);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) 
    {
      result->status = true;
      result->total_dist = distance.data;
      publisher_->publish(distance);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded with total distance: %f", result->total_dist);      
    }
  }
}; // class MyActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
