#include "teb_local_planner/teb_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace teb_local_planner
{

void TebController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".max_vel_x",
    rclcpp::ParameterValue(max_vel_x_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".max_vel_theta",
    rclcpp::ParameterValue(max_vel_theta_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".acc_lim_x",
    rclcpp::ParameterValue(acc_lim_x_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".acc_lim_theta",
    rclcpp::ParameterValue(acc_lim_theta_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".teb_dt_ref",
    rclcpp::ParameterValue(teb_dt_ref_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".obstacle_min_dist",
    rclcpp::ParameterValue(obstacle_min_dist_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".weight_obstacle",
    rclcpp::ParameterValue(weight_obstacle_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".weight_viapoint",
    rclcpp::ParameterValue(weight_viapoint_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".weight_goal",
    rclcpp::ParameterValue(weight_goal_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".weight_smoothness",
    rclcpp::ParameterValue(weight_smoothness_));

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name_ + ".robot_radius",
    rclcpp::ParameterValue(robot_radius_));

  node->get_parameter(
    plugin_name_ + ".max_vel_x",
    max_vel_x_);

  node->get_parameter(
    plugin_name_ + ".max_vel_theta",
    max_vel_theta_);

  node->get_parameter(
    plugin_name_ + ".acc_lim_x",
    acc_lim_x_);

  node->get_parameter(
    plugin_name_ + ".acc_lim_theta",
    acc_lim_theta_);

  node->get_parameter(
    plugin_name_ + ".teb_dt_ref",
    teb_dt_ref_);

  node->get_parameter(
    plugin_name_ + ".obstacle_min_dist",
    obstacle_min_dist_);

  node->get_parameter(
    plugin_name_ + ".weight_obstacle",
    weight_obstacle_);

  node->get_parameter(
    plugin_name_ + ".weight_viapoint",
    weight_viapoint_);

  node->get_parameter(
    plugin_name_ + ".weight_goal",
    weight_goal_);

  node->get_parameter(
    plugin_name_ + ".weight_smoothness",
    weight_smoothness_);

  node->get_parameter(
    plugin_name_ + ".robot_radius",
    robot_radius_);

  RCLCPP_INFO(
    logger_,
    "TEB controller configured: max_vel_x=%.2f max_vel_theta=%.2f",
    max_vel_x_,
    max_vel_theta_);
}

void TebController::cleanup()
{
  trajectory_.points.clear();
  global_plan_.poses.clear();
  active_ = false;
}

void TebController::activate()
{
  active_ = true;
}

void TebController::deactivate()
{
  active_ = false;
}

void TebController::setPlan(
  const nav_msgs::msg::Path & path)
{
  global_plan_ = path;

  if (global_plan_.poses.empty()) {
    trajectory_.points.clear();
    return;
  }

  goal_pose_ = global_plan_.poses.back();
}

void TebController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

double TebController::normalizeAngle(
  double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

double TebController::distance(
  double x1,
  double y1,
  double x2,
  double y2) const
{
  return std::hypot(
    x2 - x1,
    y2 - y1);
}

double TebController::getDistanceToObstacle(
  double x,
  double y) const
{
  if (!costmap_ros_) {
    return std::numeric_limits<double>::max();
  }

  auto * costmap =
    costmap_ros_->getCostmap();

  unsigned int mx;
  unsigned int my;

  if (!costmap->worldToMap(x, y, mx, my)) {
    return 0.0;
  }

  const unsigned int radius_cells =
    static_cast<unsigned int>(
      std::ceil(
        obstacle_min_dist_
        / costmap->getResolution()));

  double best_distance =
    std::numeric_limits<double>::max();

  const int min_x =
    std::max(
      0,
      static_cast<int>(mx)
      - static_cast<int>(radius_cells));

  const int max_x =
    std::min(
      static_cast<int>(
        costmap->getSizeInCellsX()) - 1,
      static_cast<int>(mx)
      + static_cast<int>(radius_cells));

  const int min_y =
    std::max(
      0,
      static_cast<int>(my)
      - static_cast<int>(radius_cells));

  const int max_y =
    std::min(
      static_cast<int>(
        costmap->getSizeInCellsY()) - 1,
      static_cast<int>(my)
      + static_cast<int>(radius_cells));

  for (int cy = min_y; cy <= max_y; ++cy) {

    for (int cx = min_x; cx <= max_x; ++cx) {

      const unsigned char cost =
        costmap->getCost(
          static_cast<unsigned int>(cx),
          static_cast<unsigned int>(cy));

      if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }

      double ox;
      double oy;

      costmap->mapToWorld(
        static_cast<unsigned int>(cx),
        static_cast<unsigned int>(cy),
        ox,
        oy);

      best_distance =
        std::min(
          best_distance,
          distance(x, y, ox, oy));
    }
  }

  return best_distance;
}

bool TebController::isCollision(
  double x,
  double y) const
{
  return getDistanceToObstacle(
    x,
    y) <
    robot_radius_;
}

double TebController::calculateObstacleCost(
  const TrajectoryPoint & point) const
{
  const double obstacle_distance =
    getDistanceToObstacle(
      point.x,
      point.y);

  if (obstacle_distance <= robot_radius_) {
    return 10000.0;
  }

  if (
    obstacle_distance >=
    robot_radius_ + obstacle_min_dist_)
  {
    return 0.0;
  }

  const double error =
    robot_radius_
    + obstacle_min_dist_
    - obstacle_distance;

  return error * error;
}

double TebController::calculatePathCost(
  const TrajectoryPoint & point) const
{
  if (global_plan_.poses.empty()) {
    return 0.0;
  }

  double minimum_distance =
    std::numeric_limits<double>::max();

  for (const auto & pose :
       global_plan_.poses)
  {
    const double d =
      distance(
        point.x,
        point.y,
        pose.pose.position.x,
        pose.pose.position.y);

    minimum_distance =
      std::min(
        minimum_distance,
        d);
  }

  return minimum_distance * minimum_distance;
}

double TebController::calculateGoalCost(
  const TrajectoryPoint & point) const
{
  return distance(
    point.x,
    point.y,
    goal_pose_.pose.position.x,
    goal_pose_.pose.position.y);
}

double TebController::calculateSmoothnessCost(
  const TrajectoryPoint & point,
  const TrajectoryPoint & previous,
  const TrajectoryPoint & next) const
{
  const double dx1 =
    point.x - previous.x;

  const double dy1 =
    point.y - previous.y;

  const double dx2 =
    next.x - point.x;

  const double dy2 =
    next.y - point.y;

  const double angle1 =
    std::atan2(dy1, dx1);

  const double angle2 =
    std::atan2(dy2, dx2);

  const double angle_difference =
    normalizeAngle(
      angle2 - angle1);

  return angle_difference
    * angle_difference;
}

void TebController::initializeTrajectory(
  const geometry_msgs::msg::PoseStamped & pose)
{
  trajectory_.points.clear();

  if (global_plan_.poses.empty()) {
    return;
  }

  std::vector<
    geometry_msgs::msg::PoseStamped>
    selected;

  double accumulated_distance = 0.0;

  geometry_msgs::msg::PoseStamped previous =
    pose;

  selected.push_back(pose);

  for (const auto & waypoint :
       global_plan_.poses)
  {
    const double d =
      distance(
        previous.pose.position.x,
        previous.pose.position.y,
        waypoint.pose.position.x,
        waypoint.pose.position.y);

    accumulated_distance += d;

    previous = waypoint;

    selected.push_back(waypoint);

    if (
      accumulated_distance >=
      max_global_plan_lookahead_dist_)
    {
      break;
    }
  }

  if (selected.size() < 2) {
    selected.push_back(
      global_plan_.poses.back());
  }

  const double total_distance =
    accumulated_distance;

  double travelled = 0.0;

  for (
    std::size_t i = 0;
    i < selected.size();
    ++i)
  {
    if (i > 0) {

      travelled +=
        distance(
          selected[i - 1].pose.position.x,
          selected[i - 1].pose.position.y,
          selected[i].pose.position.x,
          selected[i].pose.position.y);
    }

    double theta = 0.0;

    if (i + 1 < selected.size()) {

      theta =
        std::atan2(
          selected[i + 1].pose.position.y -
          selected[i].pose.position.y,
          selected[i + 1].pose.position.x -
          selected[i].pose.position.x);

    } else if (i > 0) {

      theta =
        std::atan2(
          selected[i].pose.position.y -
          selected[i - 1].pose.position.y,
          selected[i].pose.position.x -
          selected[i - 1].pose.position.x);
    }

    TrajectoryPoint point;

    point.x =
      selected[i].pose.position.x;

    point.y =
      selected[i].pose.position.y;

    point.theta = theta;

    if (total_distance > 0.0) {

      point.time =
        (travelled / total_distance)
        * teb_dt_ref_
        * static_cast<double>(
          selected.size());

    } else {

      point.time =
        static_cast<double>(i)
        * teb_dt_ref_;
    }

    trajectory_.points.push_back(
      point);
  }
}

void TebController::calculateTimeIntervals()
{
  if (trajectory_.points.size() < 2) {
    return;
  }

  trajectory_.points[0].time = 0.0;

  for (
    std::size_t i = 1;
    i < trajectory_.points.size();
    ++i)
  {
    const double d =
      distance(
        trajectory_.points[i - 1].x,
        trajectory_.points[i - 1].y,
        trajectory_.points[i].x,
        trajectory_.points[i].y);

    const double minimum_dt =
      d /
      std::max(
        max_vel_x_,
        0.001);

    trajectory_.points[i].time =
      trajectory_.points[i - 1].time
      + std::max(
        teb_dt_ref_,
        minimum_dt);
  }
}

void TebController::optimizeTrajectory()
{
  if (trajectory_.points.size() < 3) {
    return;
  }

  for (
    int iteration = 0;
    iteration < no_outer_iterations_;
    ++iteration)
  {
    for (
      std::size_t i = 1;
      i + 1 < trajectory_.points.size();
      ++i)
    {
      auto & point =
        trajectory_.points[i];

      const auto previous =
        trajectory_.points[i - 1];

      const auto next =
        trajectory_.points[i + 1];

      const double obstacle_cost =
        calculateObstacleCost(point);

      const double path_cost =
        calculatePathCost(point);

      const double smoothness_cost =
        calculateSmoothnessCost(
          point,
          previous,
          next);

      const double gx =
        (next.x + previous.x)
        * 0.5
        - point.x;

      const double gy =
        (next.y + previous.y)
        * 0.5
        - point.y;

      double obstacle_push_x = 0.0;
      double obstacle_push_y = 0.0;

      const double obstacle_distance =
        getDistanceToObstacle(
          point.x,
          point.y);

      if (
        obstacle_distance <
        robot_radius_
        + obstacle_min_dist_)
      {
        const double dx =
          point.x - previous.x;

        const double dy =
          point.y - previous.y;

        const double length =
          std::max(
            std::hypot(dx, dy),
            0.001);

        /*
         * Conservative lateral obstacle
         * correction.
         */

        obstacle_push_x =
          (-dy / length)
          * obstacle_cost
          * 0.01;

        obstacle_push_y =
          (dx / length)
          * obstacle_cost
          * 0.01;
      }

      /*
       * Keep the optimized trajectory close
       * to the Dijkstra path.
       */

      const double path_pull =
        std::min(
          path_cost,
          1.0)
        * 0.01;

      point.x +=
        0.05
        * weight_smoothness_
        * gx
        + obstacle_push_x
        - path_pull
        * (point.x - previous.x);

      point.y +=
        0.05
        * weight_smoothness_
        * gy
        + obstacle_push_y
        - path_pull
        * (point.y - previous.y);

      (void)smoothness_cost;
    }

    calculateTimeIntervals();
  }
}

void TebController::optimizeVelocity()
{
  if (trajectory_.points.size() < 2) {
    return;
  }

  for (
    std::size_t i = 1;
    i < trajectory_.points.size();
    ++i)
  {
    const double dt =
      std::max(
        trajectory_.points[i].time
        - trajectory_.points[i - 1].time,
        0.01);

    const double dx =
      trajectory_.points[i].x
      - trajectory_.points[i - 1].x;

    const double dy =
      trajectory_.points[i].y
      - trajectory_.points[i - 1].y;

    const double velocity =
      std::hypot(dx, dy) / dt;

    if (velocity > max_vel_x_) {

      const double scale =
        max_vel_x_ / velocity;

      trajectory_.points[i].x =
        trajectory_.points[i - 1].x
        + dx * scale;

      trajectory_.points[i].y =
        trajectory_.points[i - 1].y
        + dy * scale;
    }
  }
}

geometry_msgs::msg::TwistStamped
TebController::generateVelocityCommand(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TwistStamped command;

  command.header.stamp =
    clock_->now();

  command.header.frame_id =
    pose.header.frame_id;

  if (
    global_plan_.poses.empty())
  {
    return command;
  }

  /*
   * ================================================================
   * FIND CLOSEST POINT ON THE ORIGINAL DIJKSTRA PLAN
   * ================================================================
   *
   * The Dijkstra plan is the global collision-aware route.
   *
   * We use it directly to select a lookahead target instead
   * of allowing the optimizer to create a target that is
   * unexpectedly behind or beside the robot.
   */

  std::size_t closest_index = 0;

  double closest_distance =
    std::numeric_limits<double>::max();

  for (
    std::size_t i = 0;
    i < global_plan_.poses.size();
    ++i)
  {
    const double d =
      distance(
        pose.pose.position.x,
        pose.pose.position.y,
        global_plan_.poses[i].pose.position.x,
        global_plan_.poses[i].pose.position.y);

    if (d < closest_distance) {

      closest_distance = d;

      closest_index = i;
    }
  }

  /*
   * ================================================================
   * SELECT LOOKAHEAD POINT
   * ================================================================
   */

  constexpr double LOOKAHEAD_DISTANCE = 0.25;

  std::size_t target_index =
    closest_index;

  double accumulated_distance = 0.0;

  for (
    std::size_t i = closest_index + 1;
    i < global_plan_.poses.size();
    ++i)
  {
    accumulated_distance +=
      distance(
        global_plan_.poses[i - 1]
          .pose.position.x,
        global_plan_.poses[i - 1]
          .pose.position.y,
        global_plan_.poses[i]
          .pose.position.x,
        global_plan_.poses[i]
          .pose.position.y);

    target_index = i;

    if (
      accumulated_distance >=
      LOOKAHEAD_DISTANCE)
    {
      break;
    }
  }

  /*
   * Never select beyond the actual final
   * goal.
   */

  target_index =
    std::min(
      target_index,
      global_plan_.poses.size() - 1);

  const auto & target =
    global_plan_.poses[target_index];

  const double dx =
    target.pose.position.x
    - pose.pose.position.x;

  const double dy =
    target.pose.position.y
    - pose.pose.position.y;

  /*
   * ================================================================
   * CURRENT ROBOT ORIENTATION
   * ================================================================
   */

  tf2::Quaternion q;

  tf2::fromMsg(
    pose.pose.orientation,
    q);

  double roll;
  double pitch;
  double yaw;

  tf2::Matrix3x3(q).getRPY(
    roll,
    pitch,
    yaw);

  const double target_angle =
    std::atan2(
      dy,
      dx);

  const double angle_error =
    normalizeAngle(
      target_angle - yaw);

  /*
   * ================================================================
   * LINEAR VELOCITY
   * ================================================================
   */

  double linear_velocity =
    std::hypot(dx, dy)
    / std::max(
      teb_dt_ref_,
      0.1);

  linear_velocity =
    std::min(
      linear_velocity,
      max_vel_x_);

  /*
   * Apply configured speed limit.
   */

  if (speed_limit_ > 0.0) {

    if (speed_limit_percentage_) {

      linear_velocity *=
        speed_limit_ / 100.0;

    } else {

      linear_velocity =
        std::min(
          linear_velocity,
          speed_limit_);
    }
  }

  /*
   * Smoothly reduce forward velocity
   * as the heading error increases.
   */

  const double heading_factor =
    std::max(
      0.0,
      std::cos(angle_error));

  linear_velocity *=
    heading_factor;

  /*
   * Large heading error:
   *
   * - Do not drive aggressively.
   * - Still allow a very small amount of forward
   *   motion when possible so the robot can make
   *   progress around gentle corners.
   */

  if (std::abs(angle_error) > 1.4) {

    linear_velocity = 0.0;

  } else if (std::abs(angle_error) > 1.0) {

    linear_velocity *= 0.20;

  } else if (std::abs(angle_error) > 0.7) {

    linear_velocity *= 0.50;
  }

  /*
   * ================================================================
   * IMMEDIATE COLLISION PROTECTION
   * ================================================================
   *
   * Only stop for an actual immediate obstacle.
   *
   * We deliberately do NOT stop merely because the selected
   * lookahead waypoint is inside obstacle_min_dist_.
   *
   * Dijkstra already supplies an inflated collision-free route,
   * while the local costmap handles the immediate surroundings.
   */

  const double current_obstacle_distance =
    getDistanceToObstacle(
      pose.pose.position.x,
      pose.pose.position.y);

  if (
    current_obstacle_distance <
    robot_radius_)
  {
    linear_velocity = 0.0;
  }

  /*
   * ================================================================
   * ANGULAR VELOCITY
   * ================================================================
   */

  double angular_velocity =
    angle_error * 1.5;

  angular_velocity =
    std::clamp(
      angular_velocity,
      -max_vel_theta_,
      max_vel_theta_);

  command.twist.linear.x =
    linear_velocity;

  command.twist.angular.z =
    angular_velocity;

  return command;
}

geometry_msgs::msg::TwistStamped
TebController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped command;

  command.header.stamp =
    clock_->now();

  command.header.frame_id =
    pose.header.frame_id;

  if (
    !active_
    || global_plan_.poses.empty())
  {
    return command;
  }

  initializeTrajectory(
    pose);

  optimizeTrajectory();

  optimizeVelocity();

  return generateVelocityCommand(
    pose);
}

}  // namespace teb_local_planner

PLUGINLIB_EXPORT_CLASS(
  teb_local_planner::TebController,
  nav2_core::Controller)
