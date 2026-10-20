#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cmath>
#include <memory>


class FollowGoalGenerator : public rclcpp::Node
{
public:

  FollowGoalGenerator()
  : Node("follow_goal_generator")
  {
    this->declare_parameter<double>("follow_distance", 0.8);

    follow_distance_ =
      this->get_parameter("follow_distance").as_double();

    person_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/person_pose",
        10,
        std::bind(
          &FollowGoalGenerator::personPoseCallback,
          this,
          std::placeholders::_1));

    follow_enabled_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
        "/follow_person",
        10,
        std::bind(
          &FollowGoalGenerator::followEnabledCallback,
          this,
          std::placeholders::_1));

    follow_goal_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/person_follow_goal",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Follow goal generator started.");

    RCLCPP_INFO(
      this->get_logger(),
      "Follow distance: %.2f m",
      follow_distance_);
  }


private:

  void followEnabledCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    follow_enabled_ = msg->data;
  }


  void personPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!follow_enabled_)
    {
      return;
    }

    const double qz = msg->pose.orientation.z;
    const double qw = msg->pose.orientation.w;

    const double yaw =
      2.0 * std::atan2(qz, qw);

    /*
     * Person's forward direction:
     *
     *     dx = cos(yaw)
     *     dy = sin(yaw)
     *
     * We place the robot's target behind the person.
     */

    geometry_msgs::msg::PoseStamped goal;

    goal.header.stamp =
      this->get_clock()->now();

    goal.header.frame_id =
      msg->header.frame_id;

    goal.pose.position.x =
      msg->pose.position.x -
      follow_distance_ * std::cos(yaw);

    goal.pose.position.y =
      msg->pose.position.y -
      follow_distance_ * std::sin(yaw);

    goal.pose.position.z = 0.0;

    /*
     * Point the robot toward the person.
     */

    const double target_yaw =
      std::atan2(
        msg->pose.position.y -
          goal.pose.position.y,
        msg->pose.position.x -
          goal.pose.position.x);

    goal.pose.orientation.z =
      std::sin(target_yaw / 2.0);

    goal.pose.orientation.w =
      std::cos(target_yaw / 2.0);

    follow_goal_pub_->publish(goal);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Follow target: (%.2f, %.2f)",
      goal.pose.position.x,
      goal.pose.position.y);
  }


  double follow_distance_;

  bool follow_enabled_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    person_pose_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    follow_enabled_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    follow_goal_pub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<FollowGoalGenerator>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
