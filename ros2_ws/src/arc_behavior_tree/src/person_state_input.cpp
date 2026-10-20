#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;


class PersonStateInput : public rclcpp::Node
{
public:

    PersonStateInput()
    : Node("person_state_input"),
      moving_(false),
      confused_(false),
      emergency_(false),
      running_(true)
    {
        moving_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/person_moving",
                10);

        confused_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/person_confused",
                10);

        emergency_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/emergency_stop",
                10);


        /*
         * Explicit selection, published alongside the Bools.
         *
         * "NONE" until the operator has chosen a state. The BT uses
         * this to clear a latched EMERGENCY only on an explicit new
         * selection (a restarted input node publishes NONE, which
         * does not clear it).
         */
        state_pub_ =
            this->create_publisher<std_msgs::msg::String>(
                "/person_state",
                10);


        timer_ =
            this->create_wall_timer(
                200ms,
                std::bind(
                    &PersonStateInput::publishState,
                    this));


        input_thread_ =
            std::thread(
                &PersonStateInput::inputLoop,
                this);


        RCLCPP_INFO(
            this->get_logger(),
            "Person State Input started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for person state selection.");
    }


    ~PersonStateInput()
    {
        running_ = false;

        if (input_thread_.joinable())
        {
            input_thread_.detach();
        }
    }


private:

    void inputLoop()
    {
        while (rclcpp::ok() && running_)
        {
            std::cout << std::endl;
            std::cout << "========================================"
                      << std::endl;
            std::cout << "          PERSON STATE INPUT"
                      << std::endl;
            std::cout << "========================================"
                      << std::endl;
            std::cout << "1. Follow   (person walks, cart follows)"
                      << std::endl;
            std::cout << "2. Waiting  (cart holds)"
                      << std::endl;
            std::cout << "3. Confused (cart leads, reduced speed)"
                      << std::endl;
            std::cout << "4. Emergency Stop (latched)"
                      << std::endl;
            std::cout << "========================================"
                      << std::endl;
            std::cout << "Enter state: "
                      << std::flush;


            std::string input;

            if (!std::getline(std::cin, input))
            {
                break;
            }


            if (input == "1")
            {
                moving_ = true;
                confused_ = false;
                emergency_ = false;
                setSelected("FOLLOW");

                RCLCPP_INFO(
                    this->get_logger(),
                    "PERSON STATE: FOLLOW (moving)");
            }
            else if (input == "2")
            {
                moving_ = false;
                confused_ = false;
                emergency_ = false;
                setSelected("WAIT");

                RCLCPP_INFO(
                    this->get_logger(),
                    "PERSON STATE: WAITING");
            }
            else if (input == "3")
            {
                moving_ = false;
                confused_ = true;
                emergency_ = false;
                setSelected("CONFUSED");

                RCLCPP_INFO(
                    this->get_logger(),
                    "PERSON STATE: CONFUSED");
            }
            else if (input == "4")
            {
                moving_ = false;
                confused_ = false;
                emergency_ = true;
                setSelected("EMERGENCY");

                RCLCPP_WARN(
                    this->get_logger(),
                    "PERSON STATE: EMERGENCY STOP");
            }
            else
            {
                std::cout
                    << "Invalid choice. Enter 1, 2, 3, or 4."
                    << std::endl;
            }
        }
    }


    void publishState()
    {
        std_msgs::msg::Bool msg;


        msg.data = moving_.load();
        moving_pub_->publish(msg);


        msg.data = confused_.load();
        confused_pub_->publish(msg);


        msg.data = emergency_.load();
        emergency_pub_->publish(msg);


        std_msgs::msg::String state_msg;
        {
            std::lock_guard<std::mutex> lock(selected_mutex_);
            state_msg.data = selected_;
        }
        state_pub_->publish(state_msg);
    }


    void setSelected(const std::string& state)
    {
        std::lock_guard<std::mutex> lock(selected_mutex_);
        selected_ = state;
    }


    std::atomic<bool> moving_;
    std::atomic<bool> confused_;
    std::atomic<bool> emergency_;
    std::atomic<bool> running_;

    std::mutex selected_mutex_;
    std::string selected_ = "NONE";


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        moving_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        confused_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        emergency_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        state_pub_;


    rclcpp::TimerBase::SharedPtr timer_;

    std::thread input_thread_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PersonStateInput>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
