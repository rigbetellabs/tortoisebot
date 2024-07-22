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

#ifndef V4L2_CAMERA__PARAMETERS_HPP_
#define V4L2_CAMERA__PARAMETERS_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#include <v4l2_camera/v4l2_camera_device.hpp>

namespace rclcpp {namespace node_interfaces {struct PostSetParametersCallbackHandle;}}

namespace v4l2_camera
{
class Parameters
{
public:
  Parameters(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface);

  /** Declare read-only parameters */
  void declareStaticParameters();

  /** Declare parameters for output formats */
  void declareOutputParameters();

  /** Declare parameters by querying device */
  void declareDeviceParameters(V4l2CameraDevice const & device);

  /** Set callbacks for inspecting and applying parameter changes */
  void setParameterChangedCallback(std::function<void(rclcpp::Parameter)> callback);

  rclcpp::Parameter getParameter(std::string const & name) const
  {
    return parameters_interface_->get_parameter(name);
  }

  const rclcpp::ParameterValue & getParameterValue(std::string const & name) const
  {
    return parameters_interface_->get_parameter(name).get_parameter_value();
  }

  template<typename T>
  decltype(auto) getValue(std::string const & name) const
  {
    return parameters_interface_->get_parameter(name).get_value<T>();
  }

  std::string getVideoDevice() const {return getValue<std::string>("video_device");}
  std::string getCameraInfoUrl() const {return getValue<std::string>("camera_info_url");}
  std::string getCameraFrameId() const {return getValue<std::string>("camera_frame_id");}

  std::string getOutputEncoding() const {return getValue<std::string>("output_encoding");}

  std::string getPixelFormat() const {return getValue<std::string>("pixel_format");}
  std::vector<int64_t> getImageSize() const {return getValue<std::vector<int64_t>>("image_size");}

  std::vector<rclcpp::Parameter> getControlParameters() const
  {
    auto names = std::vector<std::string>{};
    std::transform(
      control_name_to_id_.begin(), control_name_to_id_.end(),
      std::back_inserter(names),
      [](auto kv) {return kv.first;});

    return parameters_interface_->get_parameters(names);
  }

  bool isControlParameter(rclcpp::Parameter const & parameter) const
  {
    return control_name_to_id_.find(parameter.get_name()) != control_name_to_id_.end();
  }
  int32_t getControlId(std::string const & name) const {return control_name_to_id_.at(name);}
  int32_t getControlId(rclcpp::Parameter const & param) const
  {
    return getControlId(param.get_name());
  }

private:
  using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;
  using NodeParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;
  using NodeTopicsInterface = rclcpp::node_interfaces::NodeTopicsInterface;

  NodeLoggingInterface::SharedPtr logging_interface_;
  NodeParametersInterface::SharedPtr parameters_interface_;
  NodeTopicsInterface::SharedPtr topics_interface_;

  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle>
  on_set_parameter_callback_handle_;
  std::shared_ptr<rclcpp::node_interfaces::PostSetParametersCallbackHandle>
  post_set_parameter_callback_handle_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  std::function<void(rclcpp::Parameter)> parameter_changed_callback_;

  std::unordered_map<std::string, int32_t> control_name_to_id_;

  void declareFormatParameters(V4l2CameraDevice const & device);
  void declareControlParameters(V4l2CameraDevice const & device);

  inline rcl_interfaces::msg::ParameterDescriptor make_descriptor(
    std::string description,
    std::string additional_constraints, bool read_only)
  {
    auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    parameter_descriptor.description = std::move(description);
    parameter_descriptor.additional_constraints = std::move(additional_constraints);
    parameter_descriptor.read_only = read_only;
    return parameter_descriptor;
  }

  template<typename T>
  void declareParameter(
    std::string const & name, T value,
    rcl_interfaces::msg::ParameterDescriptor const & parameter_descriptor)
  {
    auto parameter_value = rclcpp::ParameterValue{value};
    parameters_interface_->declare_parameter(name, parameter_value, parameter_descriptor);
  }

  template<typename T>
  void declareParameter(
    std::string const & name, T value, std::string const & description,
    std::string const & additional_constraints,
    bool read_only = false)
  {
    auto parameter_descriptor = make_descriptor(description, additional_constraints, read_only);
    declareParameter(name, value, parameter_descriptor);
  }

  template<typename T>
  void declareParameter(
    std::string const & name, T value, std::string const & description,
    bool read_only = false)
  {
    declareParameter(name, value, description, "", read_only);
  }

  /** SFINAE helper to check whether add_post_set_parameters_callback is supported */
  template<typename T>
  struct hasAddPostSetParametersCallback
  {
    template<typename A>
    static std::true_type test(decltype(&A::add_post_set_parameters_callback));

    template<typename A>
    static std::false_type test(...);

    static const bool value = decltype(test<T>(nullptr))::value;
  };

  // Humble variant using AsyncParametersClient::on_parameter_event
  template<typename T>
  std::enable_if_t<!hasAddPostSetParametersCallback<T>::value> setParameterChangedCallbackImpl(
    std::shared_ptr<T>/*parameters_interface*/,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    std::function<void(rclcpp::Parameter)> callback)
  {
    parameter_event_sub_ = rclcpp::AsyncParametersClient::on_parameter_event(
      topics_interface,
      [this, callback = std::move(callback)](
        rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event) {
        if (event->node !=
        topics_interface_->get_node_base_interface()->get_fully_qualified_name())
        {
          return;
        }
        for (auto const & parameter : event->changed_parameters) {
          callback(parameters_interface_->get_parameter(parameter.name));
        }
      });
  }

  // Post-Humble variant using add_post_set_parameters_callback
  template<typename T>
  std::enable_if_t<hasAddPostSetParametersCallback<T>::value> setParameterChangedCallbackImpl(
    std::shared_ptr<T> parameters_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr /*topics_interface*/,
    std::function<void(rclcpp::Parameter)> callback)
  {
    post_set_parameter_callback_handle_ = parameters_interface->add_post_set_parameters_callback(
      [this, callback = std::move(callback)](
        std::vector<rclcpp::Parameter> const & parameters) {
        for (auto const & parameter : parameters) {
          callback(parameter);
        }
      });
  }
};
}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__PARAMETERS_HPP_
