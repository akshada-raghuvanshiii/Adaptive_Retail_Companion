/*
 * arc_motion_gate
 *
 * The single publisher of /cmd_vel. It sits between the navigation
 * controller (publishing /cmd_vel_nav) and the robot.
 *
 * The Behaviour Tree selects the execution policy:
 *   FOLLOW    : pass through the navigation velocity command.
 *   LEAD      : limit forward/reverse linear speed to lead_max_speed.
 *   WAIT      : publish zero velocity continuously.
 *   EMERGENCY : publish zero velocity immediately and latch the stop
 *               until the direct E-stop is released and the BT reports
 *               a different mode.
 *   stale/missing BT mode: treated as WAIT.
 *
 * Person tracking is intentionally not required by this node. FOLLOW and
 * LEAD are navigation execution modes until a real person tracker is
 * integrated. This node does not change the path or publish navigation goals.
 */

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class ArcMotionGate : public rclcpp::Node
{
public:
    ArcMotionGate()
    : Node("arc_motion_gate"),
      steady_(RCL_STEADY_TIME)
    {
        lead_max_speed_ = declare_parameter<double>("lead_max_speed", 0.10);
        mode_timeout_ = declare_parameter<double>("mode_timeout", 0.5);
        cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
        robot_frame_ = declare_parameter<std::string>("robot_frame", "base_footprint");

        const auto in_topic =
            declare_parameter<std::string>("input_cmd_topic", "/cmd_vel_nav");
        const auto out_topic =
            declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");

        cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(out_topic, 10);
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/arc/motion_gate_status", 10);

        cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            in_topic, 10,
            [this](geometry_msgs::msg::TwistStamped::SharedPtr msg)
            {
                last_cmd_ = *msg;
                last_cmd_time_ = steady_.now();
                have_cmd_ = true;
                publishGated();
            });

        mode_sub_ = create_subscription<std_msgs::msg::String>(
            "/arc/behavior_mode", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                mode_ = msg->data;
                mode_time_ = steady_.now();
                have_mode_ = true;
                updateEmergencyLatch();

                // Stop immediately when the BT selects EMERGENCY.
                if (estop_latched_)
                {
                    publishZero();
                }
            });

        estop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                direct_estop_ = msg->data;
                updateEmergencyLatch();
                if (estop_latched_)
                {
                    // Do not wait for the periodic timer.
                    publishZero();
                }
            });

        timer_ = create_wall_timer(50ms, [this]() { onTimer(); });
        status_timer_ = create_wall_timer(500ms, [this]() { publishStatus(); });

        RCLCPP_INFO(
            get_logger(),
            "ARC motion gate started: %s -> %s; LEAD speed cap %.2f m/s",
            in_topic.c_str(), out_topic.c_str(), lead_max_speed_);
    }

private:
    bool modeFresh() const
    {
        return have_mode_ &&
            (steady_.now() - mode_time_).seconds() < mode_timeout_;
    }

    void updateEmergencyLatch()
    {
        if (direct_estop_ || (have_mode_ && mode_ == "EMERGENCY"))
        {
            if (!estop_latched_)
            {
                RCLCPP_ERROR(get_logger(), "EMERGENCY: cart stopped (latched).");
            }
            estop_latched_ = true;
            return;
        }

        // Preserve the existing reset rule: the direct E-stop must be
        // released and the BT must actively report a non-emergency mode.
        if (estop_latched_ && modeFresh() && mode_ != "EMERGENCY" && !direct_estop_)
        {
            estop_latched_ = false;
            RCLCPP_WARN(
                get_logger(), "EMERGENCY cleared, BT mode is now %s.", mode_.c_str());
        }
    }

    std::string effectiveMode() const
    {
        if (estop_latched_)
        {
            return "EMERGENCY";
        }
        if (!modeFresh())
        {
            return "WAIT";
        }
        return mode_;
    }

    void publishGated()
    {
        const std::string mode = effectiveMode();
        last_effective_mode_ = mode;

        if (!have_cmd_ ||
            (steady_.now() - last_cmd_time_).seconds() > cmd_timeout_)
        {
            return;  // Timer handles stale commands and hard-stop states.
        }

        double scale = 1.0;
        reason_ = "FOLLOW: navigation command passed through";

        if (mode == "WAIT" || mode == "EMERGENCY")
        {
            scale = 0.0;
            reason_ = mode;
        }
        else if (mode == "LEAD")
        {
            const double vx = std::fabs(last_cmd_.twist.linear.x);
            if (vx > lead_max_speed_ && vx > 1e-9)
            {
                scale = lead_max_speed_ / vx;
            }
            reason_ = "LEAD: navigation command speed-limited";
        }
        else if (mode != "FOLLOW")
        {
            // Unknown modes fail safe: never pass motion through.
            scale = 0.0;
            reason_ = "Unknown mode: stopped";
        }

        geometry_msgs::msg::TwistStamped out = last_cmd_;
        out.header.stamp = now();
        out.twist.linear.x *= scale;
        out.twist.linear.y *= scale;
        out.twist.linear.z *= scale;
        out.twist.angular.x *= scale;
        out.twist.angular.y *= scale;
        out.twist.angular.z *= scale;

        last_scale_ = scale;
        publish(out);
    }

    void onTimer()
    {
        const std::string mode = effectiveMode();
        last_effective_mode_ = mode;

        const bool cmd_stale =
            !have_cmd_ ||
            (steady_.now() - last_cmd_time_).seconds() > cmd_timeout_;

        if (mode == "EMERGENCY" || mode == "WAIT")
        {
            reason_ = mode;
            last_scale_ = 0.0;
            publishZero();
            return;
        }

        // If the navigation controller stops sending commands, ensure the
        // robot is not left moving on its last nonzero command.
        if (cmd_stale && last_output_nonzero_)
        {
            reason_ = "Navigation command stale: stopped";
            last_scale_ = 0.0;
            publishZero();
        }
    }

    void publishZero()
    {
        geometry_msgs::msg::TwistStamped zero;
        zero.header.stamp = now();
        zero.header.frame_id = robot_frame_;
        publish(zero);
    }

    void publish(const geometry_msgs::msg::TwistStamped & msg)
    {
        last_output_nonzero_ =
            std::fabs(msg.twist.linear.x) > 1e-6 ||
            std::fabs(msg.twist.linear.y) > 1e-6 ||
            std::fabs(msg.twist.linear.z) > 1e-6 ||
            std::fabs(msg.twist.angular.x) > 1e-6 ||
            std::fabs(msg.twist.angular.y) > 1e-6 ||
            std::fabs(msg.twist.angular.z) > 1e-6;
        cmd_pub_->publish(msg);
    }

    void publishStatus()
    {
        std_msgs::msg::String status;
        std::ostringstream ss;
        ss.precision(2);
        ss << std::fixed
           << "mode=" << last_effective_mode_
           << " scale=" << last_scale_
           << " | " << reason_;
        status.data = ss.str();
        status_pub_->publish(status);

        if (status.data != last_status_text_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000, "%s", status.data.c_str());
            last_status_text_ = status.data;
        }
    }

    rclcpp::Clock steady_;

    double lead_max_speed_{0.10};
    double mode_timeout_{0.5};
    double cmd_timeout_{0.3};
    std::string robot_frame_;

    geometry_msgs::msg::TwistStamped last_cmd_;
    rclcpp::Time last_cmd_time_{0, 0, RCL_STEADY_TIME};
    bool have_cmd_{false};

    std::string mode_{"NONE"};
    rclcpp::Time mode_time_{0, 0, RCL_STEADY_TIME};
    bool have_mode_{false};

    bool direct_estop_{false};
    bool estop_latched_{false};
    bool last_output_nonzero_{false};

    std::string last_effective_mode_{"WAIT"};
    std::string reason_{"starting"};
    std::string last_status_text_;
    double last_scale_{0.0};

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArcMotionGate>());
    rclcpp::shutdown();
    return 0;
}
