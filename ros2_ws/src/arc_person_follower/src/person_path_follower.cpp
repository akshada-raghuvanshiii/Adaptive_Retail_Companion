#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class PersonPathFollower : public rclcpp::Node
{
public:
  PersonPathFollower()
  : Node("person_path_follower"),
    person_x_(0.0),
    person_y_(0.7),
    waypoint_index_(0),
    have_path_(false)
  {
    this->declare_parameter<double>("person_x", 0.0);
    this->declare_parameter<double>("person_y", 0.7);
    this->declare_parameter<double>("walk_speed", 0.30);
    this->declare_parameter<double>("waypoint_tolerance", 0.12);

    person_x_ = this->get_parameter("person_x").as_double();
    person_y_ = this->get_parameter("person_y").as_double();

    walk_speed_ =
      this->get_parameter("walk_speed").as_double();

    waypoint_tolerance_ =
      this->get_parameter("waypoint_tolerance").as_double();

    path_sub_ =
      this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path",
        rclcpp::QoS(10),
        std::bind(
          &PersonPathFollower::path_callback,
          this,
          std::placeholders::_1));

    person_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/person_pose",
        10);

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(
        &PersonPathFollower::update,
        this));

    RCLCPP_INFO(
      this->get_logger(),
      "Person path follower started at (%.2f, %.2f)",
      person_x_,
      person_y_);

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for /planned_path...");
  }

private:

  void path_callback(
    const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (msg->poses.empty()) {
      return;
    }

    path_ = msg->poses;

    waypoint_index_ = 0;
    have_path_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Received planned path with %zu waypoints",
      path_.size());
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!have_path_ || path_.empty()) {
      publish_pose();
      return;
    }

    if (waypoint_index_ >= path_.size()) {
      publish_pose();
      return;
    }

    const auto & target =
      path_[waypoint_index_].pose.position;

    const double dx =
      target.x - person_x_;

    const double dy =
      target.y - person_y_;

    const double distance =
      std::hypot(dx, dy);

    if (distance <= waypoint_tolerance_) {

      waypoint_index_++;

      if (waypoint_index_ >= path_.size()) {

        person_x_ = target.x;
        person_y_ = target.y;

        RCLCPP_INFO(
          this->get_logger(),
          "Person reached end of planned path");

        publish_pose();
        return;
      }
    }

    if (waypoint_index_ >= path_.size()) {
      publish_pose();
      return;
    }

    const auto & next_target =
      path_[waypoint_index_].pose.position;

    const double next_dx =
      next_target.x - person_x_;

    const double next_dy =
      next_target.y - person_y_;

    const double next_distance =
      std::hypot(next_dx, next_dy);

    if (next_distance > 0.001) {

      const double dt = 0.1;

      const double step =
        walk_speed_ * dt;

      const double ratio =
        std::min(1.0, step / next_distance);

      person_x_ += next_dx * ratio;
      person_y_ += next_dy * ratio;
    }

    publish_pose();
  }

  void publish_pose()
  {
    geometry_msgs::msg::PoseStamped msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    msg.pose.position.x = person_x_;
    msg.pose.position.y = person_y_;
    msg.pose.position.z = 0.0;

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    person_pose_pub_->publish(msg);
  }

  double person_x_;
  double person_y_;

  double walk_speed_;
  double waypoint_tolerance_;

  std::vector<geometry_msgs::msg::PoseStamped> path_;

  std::size_t waypoint_index_;

  bool have_path_;

  std::mutex mutex_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
    path_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    person_pose_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PersonPathFollower>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
