// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "v4l2_camera/v4l2_camera.hpp"

#include <memory>

class ComposeTest : public rclcpp::Node
{
public:
  explicit ComposeTest(rclcpp::NodeOptions const & options)
  : rclcpp::Node{"compose_test", options}
  {
    img_sub_ =
      create_subscription<sensor_msgs::msg::Image>(
      "/image_raw",
      10,
      [this](sensor_msgs::msg::Image::UniquePtr img) {
        std::stringstream ss;
        ss << "Image message address [RECEIVE]:\t" << img.get();
        RCLCPP_DEBUG(get_logger(), "%s", ss.str().c_str());
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec{};

  auto intra_comms_options = rclcpp::NodeOptions{}.use_intra_process_comms(true);
  auto camera_node = std::make_shared<v4l2_camera::V4L2Camera>(intra_comms_options);
  auto test_node = std::make_shared<ComposeTest>(intra_comms_options);

  exec.add_node(camera_node);
  exec.add_node(test_node);

  exec.spin();

  rclcpp::shutdown();
  camera_node = nullptr;
  test_node = nullptr;

  return 0;
}
