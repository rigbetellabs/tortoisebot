#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher() : Node("robot_pose_publisher")
    {
        // Declare parameters
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("reference_frame", "map");
        this->declare_parameter<double>("publish_rate", 20.0);
        this->declare_parameter<std::string>("topic_name", "robot_pose");
        
        // Get parameters
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("reference_frame", reference_frame_);
        this->get_parameter("publish_rate", publish_rate_);
        this->get_parameter("topic_name", topic_name_);
        
        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Publisher
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_name_, 10);
        
        // Timer for continuous publishing
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
        timer_ = this->create_wall_timer(
            timer_period, 
            std::bind(&RobotPosePublisher::publish_pose_callback, this));
        
        // Status tracking
        consecutive_failures_ = 0;
        has_valid_pose_ = false;
        
        // Initialize last known pose to origin
        last_known_pose_.header.frame_id = reference_frame_;
        last_known_pose_.pose.position.x = 0.0;
        last_known_pose_.pose.position.y = 0.0;
        last_known_pose_.pose.position.z = 0.0;
        last_known_pose_.pose.orientation.x = 0.0;
        last_known_pose_.pose.orientation.y = 0.0;
        last_known_pose_.pose.orientation.z = 0.0;
        last_known_pose_.pose.orientation.w = 1.0;
        
        RCLCPP_INFO(this->get_logger(), "Robot Pose Publisher started");
        RCLCPP_INFO(this->get_logger(), "Publishing %s -> %s transform", 
                   base_frame_.c_str(), reference_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Topic: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Rate: %.1f Hz", publish_rate_);
    }

private:
    void publish_pose_callback()
    {
        try
        {
            // Look up the transform
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform(
                reference_frame_,
                base_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(0.1)
            );
            
            // Create PoseStamped message
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = reference_frame_;
            
            // Set position from transform
            pose_msg.pose.position.x = transform.transform.translation.x;
            pose_msg.pose.position.y = transform.transform.translation.y;
            pose_msg.pose.position.z = transform.transform.translation.z;
            
            // Set orientation from transform
            pose_msg.pose.orientation.x = transform.transform.rotation.x;
            pose_msg.pose.orientation.y = transform.transform.rotation.y;
            pose_msg.pose.orientation.z = transform.transform.rotation.z;
            pose_msg.pose.orientation.w = transform.transform.rotation.w;
            
            // Publish the pose
            pose_publisher_->publish(pose_msg);
            
            // Store the last known pose for fallback
            last_known_pose_ = pose_msg;
            has_valid_pose_ = true;
            
            // Reset failure counter on success
            if (consecutive_failures_ > 0)
            {
                RCLCPP_INFO(this->get_logger(), 
                           "Transform lookup successful again after %d failures", 
                           consecutive_failures_);
                consecutive_failures_ = 0;
            }
            
            last_successful_time_ = this->get_clock()->now();
        }
        catch (const tf2::TransformException& ex)
        {
            consecutive_failures_++;
            
            // Log different messages based on failure count to avoid spam
            if (consecutive_failures_ == 1)
            {
                RCLCPP_WARN(this->get_logger(), 
                           "Could not transform %s to %s: %s", 
                           base_frame_.c_str(), reference_frame_.c_str(), ex.what());
            }
            else if (consecutive_failures_ % 50 == 0)  // Log every 50 failures
            {
                RCLCPP_WARN(this->get_logger(), 
                           "Still unable to get transform after %d attempts", 
                           consecutive_failures_);
            }
            
            // Publish fallback pose
            geometry_msgs::msg::PoseStamped fallback_pose;
            fallback_pose.header.stamp = this->get_clock()->now();
            fallback_pose.header.frame_id = reference_frame_;
            
            if (has_valid_pose_)
            {
                // Use last known pose if we had a valid transform before
                fallback_pose.pose = last_known_pose_.pose;
                if (consecutive_failures_ == 1)
                {
                    RCLCPP_WARN(this->get_logger(), 
                               "Publishing last known pose due to transform failure");
                }
            }
            else
            {
                // Use origin (0,0) if we never had a valid transform
                fallback_pose.pose.position.x = 0.0;
                fallback_pose.pose.position.y = 0.0;
                fallback_pose.pose.position.z = 0.0;
                fallback_pose.pose.orientation.x = 0.0;
                fallback_pose.pose.orientation.y = 0.0;
                fallback_pose.pose.orientation.z = 0.0;
                fallback_pose.pose.orientation.w = 1.0;
                
                if (consecutive_failures_ == 1)
                {
                    RCLCPP_WARN(this->get_logger(), 
                               "Publishing origin pose (0,0) due to transform failure");
                }
            }
            
            pose_publisher_->publish(fallback_pose);
            return;
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR(this->get_logger(), 
                        "Unexpected error in pose publisher: %s", ex.what());
        }
    }
    
    // Parameters
    std::string base_frame_;
    std::string reference_frame_;
    double publish_rate_;
    std::string topic_name_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Status tracking
    int consecutive_failures_;
    rclcpp::Time last_successful_time_;
    bool has_valid_pose_;
    geometry_msgs::msg::PoseStamped last_known_pose_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    try
    {
        auto robot_pose_publisher = std::make_shared<RobotPosePublisher>();
        
        // Keep the node running
        rclcpp::spin(robot_pose_publisher);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}








