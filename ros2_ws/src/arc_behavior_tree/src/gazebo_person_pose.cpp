#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

#include <cmath>
#include <memory>
#include <string>


class GazeboPersonPose : public rclcpp::Node
{
public:

    GazeboPersonPose()
    : Node("gazebo_person_pose")
    {
        this->declare_parameter<std::string>(
            "world_name",
            "default");

        this->declare_parameter<std::string>(
            "entity_name",
            "person");

        this->get_parameter(
            "world_name",
            world_name_);

        this->get_parameter(
            "entity_name",
            entity_name_);


        /*
         * /person_pose is in the RTAB-Map "map" frame, whose origin is
         * where the robot started. Gazebo's set_pose uses the Gazebo
         * world frame. These parameters give the pose of the map
         * origin in the Gazebo world, i.e. the robot's spawn pose.
         *
         * Defaults are the TurtleBot3 world spawn pose (-2.0, -0.5, 0).
         * Verify against your Gazebo launch.
         */
        this->declare_parameter<double>(
            "map_origin_in_world_x", -2.0);

        this->declare_parameter<double>(
            "map_origin_in_world_y", -0.5);

        this->declare_parameter<double>(
            "map_origin_in_world_yaw", 0.0);

        origin_x_ =
            this->get_parameter("map_origin_in_world_x").as_double();

        origin_y_ =
            this->get_parameter("map_origin_in_world_y").as_double();

        origin_yaw_ =
            this->get_parameter("map_origin_in_world_yaw").as_double();

        RCLCPP_INFO(
            this->get_logger(),
            "map origin in Gazebo world: (%.2f, %.2f, yaw %.2f)",
            origin_x_,
            origin_y_,
            origin_yaw_);

        const std::string service_name =
            "/world/" +
            world_name_ +
            "/set_pose";

        pose_client_ =
            this->create_client<
                ros_gz_interfaces::srv::SetEntityPose>(
                service_name);

        pose_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/person_pose",
                10,
                std::bind(
                    &GazeboPersonPose::poseCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo person pose controller started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for Gazebo set_pose service: %s",
            service_name.c_str());

        wait_for_service_timer_ =
            this->create_wall_timer(
                std::chrono::seconds(1),
                [this]()
                {
                    if (pose_client_->service_is_ready())
                    {
                        if (!service_ready_)
                        {
                            service_ready_ = true;

                            RCLCPP_INFO(
                                this->get_logger(),
                                "Gazebo set_pose service is READY.");

                            wait_for_service_timer_->cancel();
                        }
                    }
                });
    }


private:

    void poseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!service_ready_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Gazebo set_pose service is not ready yet.");
            return;
        }

        /*
         * Don't flood Gazebo with requests.
         */
        if (request_pending_)
        {
            return;
        }

        auto request =
            std::make_shared<
                ros_gz_interfaces::srv::SetEntityPose::Request>();

        request->entity.name =
            entity_name_;

        request->entity.type =
            ros_gz_interfaces::msg::Entity::MODEL;

        /*
         * map -> Gazebo world
         */
        const double mx = msg->pose.position.x;
        const double my = msg->pose.position.y;

        const double c = std::cos(origin_yaw_);
        const double s = std::sin(origin_yaw_);

        const double map_yaw =
            2.0 * std::atan2(
                msg->pose.orientation.z,
                msg->pose.orientation.w);

        const double world_yaw =
            map_yaw + origin_yaw_;

        request->pose.position.x =
            origin_x_ + c * mx - s * my;

        request->pose.position.y =
            origin_y_ + s * mx + c * my;

        request->pose.position.z =
            0.0;

        request->pose.orientation.x = 0.0;

        request->pose.orientation.y = 0.0;

        request->pose.orientation.z =
            std::sin(world_yaw / 2.0);

        request->pose.orientation.w =
            std::cos(world_yaw / 2.0);

        request_pending_ = true;

        RCLCPP_DEBUG(
            this->get_logger(),
            "Moving person to (%.2f, %.2f).",
            request->pose.position.x,
            request->pose.position.y);

        pose_client_->async_send_request(
            request,
            [this](
                rclcpp::Client<
                    ros_gz_interfaces::srv::SetEntityPose>::SharedFuture
                    result)
            {
                request_pending_ = false;

                if (!result.get()->success)
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Gazebo rejected person pose update.");
                }
            });
    }


    std::string world_name_;
    std::string entity_name_;

    double origin_x_ = -2.0;
    double origin_y_ = -0.5;
    double origin_yaw_ = 0.0;

    bool service_ready_ = false;
    bool request_pending_ = false;

    rclcpp::Client<
        ros_gz_interfaces::srv::SetEntityPose>::SharedPtr
        pose_client_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_sub_;

    rclcpp::TimerBase::SharedPtr
        wait_for_service_timer_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<GazeboPersonPose>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
