#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

#include <raspicam.h>
#include <memory>

class RasPiCamPublisher : public rclcpp::Node {
public:
    RasPiCamPublisher(rclcpp::NodeOptions options = rclcpp::NodeOptions());

    ~RasPiCamPublisher();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_img_compressed;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr srv_info;

    sensor_msgs::msg::CameraInfo camera_info;

    std::shared_ptr<RASPIVID_STATE> state;

    void set_camera_info(const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
                         std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res);

    void onImageRaw(const uint8_t* start, const uint8_t* end);

    void onImageCompressed(const uint8_t* start, const uint8_t* end);

    void onMotion(const uint8_t* start, const uint8_t* end);

};  // node
