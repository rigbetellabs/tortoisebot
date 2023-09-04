#include <RasPiCamPublisherNode.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RasPiCamPublisher>());
    rclcpp::shutdown();
    return 0;
}
