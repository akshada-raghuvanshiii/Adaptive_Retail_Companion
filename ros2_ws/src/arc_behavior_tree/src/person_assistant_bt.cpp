#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>


using namespace BT;
using namespace std::chrono_literals;


/* ============================================================
 * PERSON ASSISTANT ROS NODE
 * ============================================================ */

class PersonAssistantNode : public rclcpp::Node
{
public:

    PersonAssistantNode()
    : Node("person_assistant_bt"),
      person_moving_(false),
      person_confused_(false),
      emergency_stop_(false),
      emergency_latched_(false)
    {
        person_moving_sub_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/person_moving",
                10,
                std::bind(
                    &PersonAssistantNode::personMovingCallback,
                    this,
                    std::placeholders::_1));


        person_confused_sub_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/person_confused",
                10,
                std::bind(
                    &PersonAssistantNode::personConfusedCallback,
                    this,
                    std::placeholders::_1));


        emergency_sub_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/emergency_stop",
                10,
                std::bind(
                    &PersonAssistantNode::emergencyCallback,
                    this,
                    std::placeholders::_1));


        /*
         * Explicit operator selection from person_state_input.
         * Used only to clear a latched EMERGENCY.
         */
        person_state_sub_ =
            this->create_subscription<std_msgs::msg::String>(
                "/person_state",
                10,
                std::bind(
                    &PersonAssistantNode::personStateCallback,
                    this,
                    std::placeholders::_1));


        follow_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/follow_person",
                10);


        lead_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/lead_person",
                10);


        wait_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/wait_for_person",
                10);


        emergency_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/emergency_stop_active",
                10);


        /*
         * Single behaviour-mode output:
         *   FOLLOW | WAIT | LEAD | EMERGENCY
         *
         * Read by shopping_manager (stage gate) and arc_motion_gate
         * (speed / gap policy). The four Bool topics above are kept
         * because planned_path_to_teb and fake_person_tracker use them.
         */
        mode_pub_ =
            this->create_publisher<std_msgs::msg::String>(
                "/arc/behavior_mode",
                10);


        RCLCPP_INFO(
            this->get_logger(),
            "Person Assistant Behaviour Tree started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing behaviour mode on /arc/behavior_mode "
            "(FOLLOW / WAIT / LEAD / EMERGENCY).");
    }


    bool isPersonMoving() const
    {
        return person_moving_;
    }


    bool isPersonConfused() const
    {
        return person_confused_;
    }


    /*
     * EMERGENCY is latched: once seen, it stays active until the
     * operator explicitly selects another state.
     */
    bool isEmergency() const
    {
        return emergency_stop_ || emergency_latched_;
    }


    void publishFollow()
    {
        publishState(
            true,
            false,
            false,
            false,
            "FOLLOW");
    }


    void publishLead()
    {
        publishState(
            false,
            true,
            false,
            false,
            "LEAD");
    }


    void publishWait()
    {
        publishState(
            false,
            false,
            true,
            false,
            "WAIT");
    }


    void publishEmergencyStop()
    {
        publishState(
            false,
            false,
            false,
            true,
            "EMERGENCY");
    }


private:

    void personMovingCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        person_moving_ = msg->data;
    }


    void personConfusedCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        person_confused_ = msg->data;
    }


    void emergencyCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        emergency_stop_ = msg->data;

        if (emergency_stop_ && !emergency_latched_)
        {
            emergency_latched_ = true;

            RCLCPP_ERROR(
                this->get_logger(),
                "EMERGENCY latched. Select another state "
                "explicitly to clear it.");
        }
    }


    void personStateCallback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        /*
         * Clear the latch only on an explicit non-emergency
         * selection. "NONE" (e.g. a restarted input node) does
         * not clear it.
         */
        if (!emergency_latched_ || emergency_stop_)
        {
            return;
        }

        if (msg->data == "FOLLOW" ||
            msg->data == "WAIT" ||
            msg->data == "CONFUSED")
        {
            emergency_latched_ = false;

            RCLCPP_WARN(
                this->get_logger(),
                "EMERGENCY cleared by explicit selection: %s",
                msg->data.c_str());
        }
    }


    void publishState(
        bool follow,
        bool lead,
        bool wait,
        bool emergency,
        const std::string& mode)
    {
        std_msgs::msg::String mode_msg;
        mode_msg.data = mode;
        mode_pub_->publish(mode_msg);

        if (mode != last_mode_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "BT mode: %s -> %s",
                last_mode_.c_str(),
                mode.c_str());

            last_mode_ = mode;
        }


        std_msgs::msg::Bool msg;


        msg.data = follow;
        follow_pub_->publish(msg);


        msg.data = lead;
        lead_pub_->publish(msg);


        msg.data = wait;
        wait_pub_->publish(msg);


        msg.data = emergency;
        emergency_pub_->publish(msg);
    }


    bool person_moving_;
    bool person_confused_;
    bool emergency_stop_;
    bool emergency_latched_;

    std::string last_mode_ = "NONE";


    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        person_moving_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        person_confused_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        emergency_sub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        person_state_sub_;


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        follow_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        lead_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        wait_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        emergency_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        mode_pub_;
};


/* ============================================================
 * CONDITION NODES
 * ============================================================ */

class IsMoving : public ConditionNode
{
public:

    IsMoving(
        const std::string& name,
        const NodeConfiguration& config,
        PersonAssistantNode* node)
    : ConditionNode(name, config),
      node_(node)
    {
    }


    static PortsList providedPorts()
    {
        return {};
    }


    NodeStatus tick() override
    {
        return node_->isPersonMoving()
            ? NodeStatus::SUCCESS
            : NodeStatus::FAILURE;
    }


private:

    PersonAssistantNode* node_;
};


class IsConfused : public ConditionNode
{
public:

    IsConfused(
        const std::string& name,
        const NodeConfiguration& config,
        PersonAssistantNode* node)
    : ConditionNode(name, config),
      node_(node)
    {
    }


    static PortsList providedPorts()
    {
        return {};
    }


    NodeStatus tick() override
    {
        return node_->isPersonConfused()
            ? NodeStatus::SUCCESS
            : NodeStatus::FAILURE;
    }


private:

    PersonAssistantNode* node_;
};


class IsEmergency : public ConditionNode
{
public:

    IsEmergency(
        const std::string& name,
        const NodeConfiguration& config,
        PersonAssistantNode* node)
    : ConditionNode(name, config),
      node_(node)
    {
    }


    static PortsList providedPorts()
    {
        return {};
    }


    NodeStatus tick() override
    {
        return node_->isEmergency()
            ? NodeStatus::SUCCESS
            : NodeStatus::FAILURE;
    }


private:

    PersonAssistantNode* node_;
};


/* ============================================================
 * ACTION NODES
 * ============================================================ */

class FollowPerson : public SyncActionNode
{
public:

    FollowPerson(
        const std::string& name,
        const NodeConfiguration& config,
        PersonAssistantNode* node)
    : SyncActionNode(name, config),
      node_(node)
    {
    }


    static PortsList providedPorts()
    {
        return {};
    }


    NodeStatus tick() override
    {
        node_->publishFollow();

        return NodeStatus::SUCCESS;
    }


private:

    PersonAssistantNode* node_;
};


class LeadPerson : public SyncActionNode
{
public:

    LeadPerson(
        const std::string& name,
        const NodeConfiguration& config,
        PersonAssistantNode* node)
    : SyncActionNode(name, config),
      node_(node)
    {
    }


    static PortsList providedPorts()
    {
        return {};
    }


    NodeStatus tick() override
    {
        node_->publishLead();

        return NodeStatus::SUCCESS;
    }


private:

    PersonAssistantNode* node_;
};


class WaitForPerson : public SyncActionNode
{
public:

    WaitForPerson(
        const std::string& name,
        const NodeConfiguration& config,
        PersonAssistantNode* node)
    : SyncActionNode(name, config),
      node_(node)
    {
    }


    static PortsList providedPorts()
    {
        return {};
    }


    NodeStatus tick() override
    {
        node_->publishWait();

        return NodeStatus::SUCCESS;
    }


private:

    PersonAssistantNode* node_;
};


class EmergencyStop : public SyncActionNode
{
public:

    EmergencyStop(
        const std::string& name,
        const NodeConfiguration& config,
        PersonAssistantNode* node)
    : SyncActionNode(name, config),
      node_(node)
    {
    }


    static PortsList providedPorts()
    {
        return {};
    }


    NodeStatus tick() override
    {
        node_->publishEmergencyStop();

        return NodeStatus::SUCCESS;
    }


private:

    PersonAssistantNode* node_;
};


/* ============================================================
 * MAIN
 * ============================================================ */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);


    auto node =
        std::make_shared<PersonAssistantNode>();


    BehaviorTreeFactory factory;


    factory.registerNodeType<IsMoving>(
        "IsMoving",
        node.get());

    factory.registerNodeType<IsConfused>(
        "IsConfused",
        node.get());

    factory.registerNodeType<IsEmergency>(
        "IsEmergency",
        node.get());

    factory.registerNodeType<FollowPerson>(
        "FollowPerson",
        node.get());

    factory.registerNodeType<LeadPerson>(
        "LeadPerson",
        node.get());

    factory.registerNodeType<WaitForPerson>(
        "WaitForPerson",
        node.get());

    factory.registerNodeType<EmergencyStop>(
        "EmergencyStop",
        node.get());


    std::string tree_file;


    const char* env_tree =
        std::getenv("ARC_BT_TREE");


    if (env_tree != nullptr)
    {
        tree_file = env_tree;
    }
    else
    {
        tree_file =
            ament_index_cpp::get_package_share_directory(
                "arc_behavior_tree")
            + "/behavior_trees/person_assistant.xml";
    }


    auto tree =
        factory.createTreeFromFile(tree_file);


    rclcpp::Rate rate(10.0);


    RCLCPP_INFO(
        node->get_logger(),
        "Running Behaviour Tree at 10 Hz.");


    while (rclcpp::ok())
    {
        /*
         * Process the newest person state first.
         */

        rclcpp::spin_some(node);


        /*
         * Then evaluate the Behaviour Tree.
         */

        tree.tickOnce();


        rate.sleep();
    }


    rclcpp::shutdown();

    return 0;
}
