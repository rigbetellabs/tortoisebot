#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

class GoalStatusPublisher : public rclcpp::Node
{
public:
    GoalStatusPublisher() : Node("goal_status_publisher")
    {
        // Define publisher
        goal_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("robot/nav_status", 10);

        // Define subscription
        result_sub_ = this->create_subscription
        <action_msgs::msg::GoalStatusArray>(
            "navigate_to_pose/_action/status",
            10,
            std::bind(&GoalStatusPublisher::goal_status_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Goal Status Publisher initialized");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr goal_status_pub_;
    rclcpp::Subscription
        <action_msgs::msg::GoalStatusArray>::SharedPtr result_sub_;
    int goal_count_;

    void goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        if (!msg->status_list.empty())
        {
            auto status = msg->status_list.back().status;
            if (status == 2)  // Executing status
            {
                publish_goal_status(1);
            }
            else if (status == 4)  // SUCCEEDED status
            {
                publish_goal_status(2);
            }
            else if (status == 5 || status == 6)  // Cancelled status
            {
                publish_goal_status(3);
            }
        }
    }

    void publish_goal_status(int status)
    {
        // Publish the goal status to the 'robot/nav_status' topic
        auto msg = std::make_shared<std_msgs::msg::Int32>();
        msg->data = status;
        goal_status_pub_->publish(*msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto goal_status_publisher = std::make_shared
    <GoalStatusPublisher>();
    rclcpp::spin(goal_status_publisher);
    rclcpp::shutdown();
    return 0;
}
