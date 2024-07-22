// Copyright 2022 Bold Hearts
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

#include "v4l2_camera/parameters.hpp"

#include <string>

#include "v4l2_camera/fourcc.hpp"

namespace v4l2_camera
{

Parameters::Parameters(
  NodeParametersInterface::SharedPtr parameters_interface,
  NodeTopicsInterface::SharedPtr topics_interface,
  NodeLoggingInterface::SharedPtr logging_interface)
: logging_interface_{std::move(logging_interface)},
  parameters_interface_{std::move(parameters_interface)},
  topics_interface_{std::move(topics_interface)}
{}

void Parameters::declareStaticParameters()
{
  declareParameter("video_device", "/dev/video0", "Path to video device", true);
  declareParameter(
    "camera_info_url", "", "The location for getting camera calibration data",
    true);
  declareParameter("camera_frame_id", "camera", "Frame id inserted in published image", true);
}

void Parameters::declareOutputParameters()
{
  declareParameter(
    "output_encoding", "rgb8",
    "ROS image encoding to use for the output image. "
    "Can be any supported by cv_bridge given the input pixel format");
}

void Parameters::declareDeviceParameters(V4l2CameraDevice const & device)
{
  declareFormatParameters(device);
  declareControlParameters(device);
}

void Parameters::declareFormatParameters(V4l2CameraDevice const & device)
{  //// pixel_format
  auto const & image_formats = device.getImageFormats();
  auto pixel_format_constraints = std::ostringstream{};
  for (auto const & format : image_formats) {
    pixel_format_constraints <<
      "\"" << FourCC::toString(format.pixelFormat) << "\"" <<
      " (" << format.description << "), ";
  }
  auto str = pixel_format_constraints.str();
  str = str.substr(0, str.size() - 2);
  declareParameter("pixel_format", "YUYV", "Pixel format (FourCC)", str);

  //// image_size
  using ImageSize = std::vector<int64_t>;
  auto image_size = ImageSize{};

  // List available image sizes per format
  auto const & image_sizes = device.getImageSizes();
  auto image_sizes_constraints = std::ostringstream{};
  image_sizes_constraints << "Available image sizes:";

  for (auto const & format : image_formats) {
    image_sizes_constraints << "\n" << FourCC::toString(format.pixelFormat) << " (" <<
      format.description << ")";

    // See if image sizes are available for the given format
    auto iter = image_sizes.find(format.pixelFormat);
    if (iter == image_sizes.end()) {
      RCLCPP_ERROR_STREAM(
        logging_interface_->get_logger(),
        "No sizes available to create parameter description for format: " << format.description);
      continue;
    }

    auto size_type = iter->second.first;
    auto & sizes = iter->second.second;
    switch (size_type) {
      case V4l2CameraDevice::ImageSizeType::DISCRETE:
        for (auto const & image_size : sizes) {
          image_sizes_constraints << "\n\t" << image_size.first << "x" << image_size.second;
        }
        break;
      case V4l2CameraDevice::ImageSizeType::STEPWISE:
        image_sizes_constraints << "\n\tmin:\t" << sizes[0].first << "x" << sizes[0].second;
        image_sizes_constraints << "\n\tmax:\t" << sizes[1].first << "x" << sizes[1].second;
        image_sizes_constraints << "\n\tstep:\t" << sizes[2].first << "x" << sizes[2].second;
        break;
      case V4l2CameraDevice::ImageSizeType::CONTINUOUS:
        image_sizes_constraints << "\n\tmin:\t" << sizes[0].first << "x" << sizes[0].second;
        image_sizes_constraints << "\n\tmax:\t" << sizes[1].first << "x" << sizes[1].second;
        break;
    }
  }

  declareParameter<std::vector<int64_t>>(
    "image_size", {640, 480}, "Image width & height",
    image_sizes_constraints.str());
}

void Parameters::declareControlParameters(V4l2CameraDevice const & device)
{
  // Helper to transform control name to parameter name
  // - makes all lower case
  // - removes ',', '(' and ')'
  // - replaces spaces with underscores
  auto toParamName =
    [](std::string name) {
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      name.erase(std::remove(name.begin(), name.end(), ','), name.end());
      name.erase(std::remove(name.begin(), name.end(), '('), name.end());
      name.erase(std::remove(name.begin(), name.end(), ')'), name.end());
      std::replace(name.begin(), name.end(), ' ', '_');
      return name;
    };

  for (auto const & c : device.getControls()) {
    auto name = toParamName(c.name);
    auto descriptor = make_descriptor(c.name, "", false);
    switch (c.type) {
      case ControlType::INT:
        {
          auto range = rcl_interfaces::msg::IntegerRange{};
          range.from_value = c.minimum;
          range.to_value = c.maximum;
          descriptor.integer_range.push_back(range);
          declareParameter<int64_t>(name, c.defaultValue, descriptor);
          break;
        }
      case ControlType::BOOL:
        {
          declareParameter<bool>(name, c.defaultValue != 0, descriptor);
          break;
        }
      case ControlType::MENU:
        {
          auto sstr = std::ostringstream{};
          for (auto const & o : c.menuItems) {
            sstr << o.first << " - " << o.second << ", ";
          }
          auto str = sstr.str();
          descriptor.additional_constraints = str.substr(0, str.size() - 2);
          declareParameter<int64_t>(name, c.defaultValue, descriptor);
          break;
        }
      default:
        RCLCPP_WARN(
          logging_interface_->get_logger(),
          "Control type not currently supported: %s, for control: %s",
          std::to_string(unsigned(c.type)).c_str(),
          c.name.c_str());
        continue;
    }
    control_name_to_id_[name] = c.id;
  }
}

void Parameters::setParameterChangedCallback(std::function<void(rclcpp::Parameter)> callback)
{
  // Callback for inspecting and validating changes
  // TODO(sgvandijk): validate parameters where possible, such as output format
  on_set_parameter_callback_handle_ = parameters_interface_->add_on_set_parameters_callback(
    [](std::vector<rclcpp::Parameter> const & /*parameters*/) {
      auto result = rcl_interfaces::msg::SetParametersResult{};
      result.successful = true;
      return result;
    });

  // Callback for actually applying changes
  setParameterChangedCallbackImpl(parameters_interface_, topics_interface_, callback);
}
}  // namespace v4l2_camera
