#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <memory>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class SetTargetPosition : public rclcpp::Node
{
public:
    SetTargetPosition()
        : Node("set_target_position"), gripper_is_open_(true)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt2 Control with gripper (node created).");
    }

    void initialize()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "arm");

        gripper_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "gripper");

        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        move_group_interface_->setEndEffector("gripper");
        move_group_interface_->setNumPlanningAttempts(10);
        move_group_interface_->setPlanningTime(5.0);

        arm_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/arm_command", 10,
            std::bind(&SetTargetPosition::armCommandCallback, this, std::placeholders::_1));

        gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/gripper_command", 10,
            std::bind(&SetTargetPosition::gripperCommandCallback, this, std::placeholders::_1));

        go_home_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/go_home",
            std::bind(&SetTargetPosition::goHomeService, this, std::placeholders::_1, std::placeholders::_2));

        go_stand_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/go_stand",
            std::bind(&SetTargetPosition::goStandService, this, std::placeholders::_1, std::placeholders::_2));

        gripper_toggle_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/gripper_toggle",
            std::bind(&SetTargetPosition::gripperToggleService, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Subscribers and services created.");

        // 🏠 Move to home initially
        RCLCPP_INFO(this->get_logger(), "Moving to 'home' named target...");
        bool success = moveToNamedTarget("home");
        if (!success)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to move to 'home' named target. Falling back to 'stand'.");
            moveToNamedTarget("stand");
        }

        rclcpp::sleep_for(1s);
    }

private:
    void armCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received arm command: '%s'", cmd.c_str());

        if (cmd == "home" || cmd == "stand")
        {
            bool ok = moveToNamedTarget(cmd);
            if (!ok)
                RCLCPP_ERROR(this->get_logger(), "Failed to move to named target '%s'", cmd.c_str());
        }
        else if (cmd == "example_pose")
        {
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = -0.00954723;
            target_pose.position.y = 0.16184;
            target_pose.position.z = 0.217192;
            target_pose.orientation.x = 0.707052;
            target_pose.orientation.y = -1.89616e-05;
            target_pose.orientation.z = -1.28814e-05;
            target_pose.orientation.w = 0.707162;

            bool ok = moveToPoseTarget(target_pose);
            if (!ok)
                RCLCPP_ERROR(this->get_logger(), "Failed to move to pose 'example_pose'");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown arm command: '%s'", cmd.c_str());
        }
    }

    void gripperCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received gripper command: '%s'", cmd.c_str());

        if (cmd == "open")
        {
            controlGripper(true);
        }
        else if (cmd == "close")
        {
            controlGripper(false);
        }
        else if (cmd == "toggle")
        {
            controlGripper(!gripper_is_open_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown gripper command: '%s'", cmd.c_str());
        }
    }

    void goHomeService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        bool ok = moveToNamedTarget("home");
        res->success = ok;
        res->message = ok ? "Moved to home" : "Failed to move to home";
    }

    void goStandService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        bool ok = moveToNamedTarget("stand");
        res->success = ok;
        res->message = ok ? "Moved to stand" : "Failed to move to stand";
    }

    void gripperToggleService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        controlGripper(!gripper_is_open_);
        res->success = true;
        res->message = gripper_is_open_ ? "Gripper opened" : "Gripper closed";
    }

    bool moveToNamedTarget(const std::string &name)
    {
        if (!move_group_interface_)
        {
            RCLCPP_ERROR(this->get_logger(), "Move group not initialized");
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_interface_->setNamedTarget(name);

        auto r = move_group_interface_->plan(plan);
        bool success = (r == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning to '%s' succeeded, executing...", name.c_str());
            move_group_interface_->execute(plan);
            rclcpp::sleep_for(500ms);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning to '%s' failed.", name.c_str());
        }
        return success;
    }

    bool moveToPoseTarget(const geometry_msgs::msg::Pose &pose)
    {
        if (!move_group_interface_)
        {
            RCLCPP_ERROR(this->get_logger(), "Move group not initialized");
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_interface_->setPoseTarget(pose);

        auto r = move_group_interface_->plan(plan);
        bool success = (r == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Pose planning succeeded, executing...");
            move_group_interface_->execute(plan);
            rclcpp::sleep_for(500ms);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Pose planning failed.");
        }
        return success;
    }

    void controlGripper(bool open)
    {
        if (!gripper_group_interface_)
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper group not initialized");
            return;
        }

        std::vector<double> joint_values;
        if (open)
        {
            joint_values = {0.0};
            RCLCPP_INFO(this->get_logger(), "Opening gripper...");
        }
        else
        {
            joint_values = {-1.57};
            RCLCPP_INFO(this->get_logger(), "Closing gripper...");
        }

        gripper_group_interface_->setJointValueTarget(joint_values);
        auto r = gripper_group_interface_->move();
        if (r == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Gripper moved successfully!");
            gripper_is_open_ = open;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move gripper!");
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_cmd_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_stand_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_toggle_srv_;
    bool gripper_is_open_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetTargetPosition>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}