#ifndef TEB_LOCAL_PLANNER__TEB_CONTROLLER_HPP_
#define TEB_LOCAL_PLANNER__TEB_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace teb_local_planner
{

class TebController : public nav2_core::Controller
{
public:

  TebController() = default;
  ~TebController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(
    const double & speed_limit,
    const bool & percentage) override;

private:

  struct TrajectoryPoint
  {
    double x;
    double y;
    double theta;
    double time;
  };

  struct Trajectory
  {
    std::vector<TrajectoryPoint> points;
    double cost{0.0};
  };

  void initializeTrajectory(
    const geometry_msgs::msg::PoseStamped & pose);

  void optimizeTrajectory();

  void optimizeVelocity();

  void calculateTimeIntervals();

  double calculateObstacleCost(
    const TrajectoryPoint & point) const;

  double calculatePathCost(
    const TrajectoryPoint & point) const;

  double calculateGoalCost(
    const TrajectoryPoint & point) const;

  double calculateSmoothnessCost(
    const TrajectoryPoint & point,
    const TrajectoryPoint & previous,
    const TrajectoryPoint & next) const;

  double normalizeAngle(double angle) const;

  double distance(
    double x1,
    double y1,
    double x2,
    double y2) const;

  bool isCollision(
    double x,
    double y) const;

  double getDistanceToObstacle(
    double x,
    double y) const;

  geometry_msgs::msg::TwistStamped
  generateVelocityCommand(
    const geometry_msgs::msg::PoseStamped & pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string plugin_name_;

  rclcpp::Logger logger_{rclcpp::get_logger("teb_local_planner")};

  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;

  Trajectory trajectory_;

  bool active_{false};

  double max_vel_x_{0.15};
  double max_vel_x_backwards_{0.05};
  double max_vel_theta_{1.0};

  double acc_lim_x_{0.5};
  double acc_lim_theta_{1.0};

  double min_turning_radius_{0.0};

  double teb_dt_ref_{0.3};
  double teb_dt_hysteresis_{0.1};

  double weight_obstacle_{50.0};
  double weight_viapoint_{10.0};
  double weight_goal_{20.0};
  double weight_smoothness_{5.0};

  double obstacle_min_dist_{0.20};
  double robot_radius_{0.105};

  double xy_goal_tolerance_{0.15};
  double yaw_goal_tolerance_{0.15};

  double max_global_plan_lookahead_dist_{2.0};

  int no_inner_iterations_{5};
  int no_outer_iterations_{3};

  int trajectory_points_{15};

  double control_frequency_{10.0};

  double speed_limit_{0.0};
  bool speed_limit_percentage_{false};

  geometry_msgs::msg::PoseStamped goal_pose_;
};

}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER__TEB_CONTROLLER_HPP_
