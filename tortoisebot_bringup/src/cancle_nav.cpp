#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

class Nav2GoalCanceller : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2GoalCanceller() : Node("nav2_goal_canceller")
    {
        // Create action client for Nav2
        action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // Create subscriber for cancel navigation topic
        cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "cancel_navigation", 10,
            std::bind(&Nav2GoalCanceller::cancel_callback, this, std::placeholders::_1));

        // Wait for action server to be available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 Goal Canceller node started");
            RCLCPP_INFO(this->get_logger(), "Listening on 'cancel_navigation' topic");
        }
    }

private:
    void cancel_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Received cancel signal - cancelling all Nav2 goals");
            cancel_all_goals();
        }
    }

    void cancel_all_goals()
    {
        // Cancel all goals by sending a cancel request without specific goal ID
        auto cancel_future = action_client_->async_cancel_all_goals();
        
        // Optional: Wait for the cancel response and log result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), 
                                              cancel_future, 
                                              std::chrono::seconds(5)) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            
            auto cancel_response = cancel_future.get();
            
            if (cancel_response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
                RCLCPP_INFO(this->get_logger(), "Successfully cancelled %zu goals", 
                           cancel_response->goals_canceling.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "Cancel request completed with code: %d", 
                           cancel_response->return_code);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get cancel response within timeout");
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2GoalCanceller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
