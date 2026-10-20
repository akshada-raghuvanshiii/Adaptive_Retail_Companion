#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class FakePersonTracker : public rclcpp::Node
{
public:
  FakePersonTracker()
  : Node("fake_person_tracker"),
    person_x_(0.0),
    person_y_(0.7),
    goal_x_(0.0),
    goal_y_(0.7),
    have_goal_(false),
    follow_person_(false),
    lead_person_(false),
    wait_for_person_(true),
    emergency_stop_(false)
  {
    this->declare_parameter<double>("person_x", 0.0);
    this->declare_parameter<double>("person_y", 0.7);
    this->declare_parameter<double>("walk_speed", 0.35);
    this->declare_parameter<double>("goal_tolerance", 0.10);

    person_x_ = this->get_parameter("person_x").as_double();
    person_y_ = this->get_parameter("person_y").as_double();
    goal_x_ = person_x_;
    goal_y_ = person_y_;

    walk_speed_ = this->get_parameter("walk_speed").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    person_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/person_pose", 10);

    shopping_goal_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/shopping_goal",
        state_qos,
        std::bind(
          &FakePersonTracker::shopping_goal_callback,
          this,
          std::placeholders::_1));

    follow_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
        "/follow_person",
        state_qos,
        std::bind(
          &FakePersonTracker::follow_callback,
          this,
          std::placeholders::_1));

    lead_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
        "/lead_person",
        state_qos,
        std::bind(
          &FakePersonTracker::lead_callback,
          this,
          std::placeholders::_1));

    wait_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
        "/wait_for_person",
        state_qos,
        std::bind(
          &FakePersonTracker::wait_callback,
          this,
          std::placeholders::_1));

    emergency_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop_active",
        state_qos,
        std::bind(
          &FakePersonTracker::emergency_callback,
          this,
          std::placeholders::_1));

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&FakePersonTracker::update, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Fake person tracker started at (%.2f, %.2f)",
      person_x_,
      person_y_);

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for /shopping_goal...");
  }

private:

  void shopping_goal_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    have_goal_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Person received shopping goal: (%.2f, %.2f)",
      goal_x_,
      goal_y_);
  }

  void follow_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    follow_person_ = msg->data;

    if (follow_person_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Person state: FOLLOW");
    }
  }

  void lead_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lead_person_ = msg->data;

    if (lead_person_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Person state: LEAD / CONFUSED");
    }
  }

  void wait_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    wait_for_person_ = msg->data;

    if (wait_for_person_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Person state: WAIT");
    }
  }

  void emergency_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_stop_ = msg->data;

    if (emergency_stop_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Person state: EMERGENCY");
    }
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    /*
     * The person itself is an independent simulated actor.
     *
     * It moves toward the current shopping-manager goal.
     * It does NOT follow /planned_path.
     *
     * WAIT and EMERGENCY freeze the person.
     */

    if (
      !have_goal_ ||
      wait_for_person_ ||
      emergency_stop_)
    {
      publish_pose();
      return;
    }

    const double dx = goal_x_ - person_x_;
    const double dy = goal_y_ - person_y_;

    const double distance = std::hypot(dx, dy);

    if (distance <= goal_tolerance_) {
      person_x_ = goal_x_;
      person_y_ = goal_y_;

      publish_pose();
      return;
    }

    double speed = walk_speed_;

    /*
     * FOLLOW:
     * normal walking speed.
     *
     * LEAD / CONFUSED:
     * slightly faster walking speed so the person
     * visibly progresses toward the destination.
     */
    if (lead_person_) {
      speed = walk_speed_ * 1.15;
    }

    const double dt = 0.1;

    const double step = speed * dt;

    const double ratio =
      std::min(1.0, step / distance);

    person_x_ += dx * ratio;
    person_y_ += dy * ratio;

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

  double goal_x_;
  double goal_y_;

  double walk_speed_;
  double goal_tolerance_;

  bool have_goal_;

  bool follow_person_;
  bool lead_person_;
  bool wait_for_person_;
  bool emergency_stop_;

  std::mutex mutex_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    person_pose_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    shopping_goal_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    follow_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    lead_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    wait_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    emergency_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<FakePersonTracker>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
