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

#ifndef V4L2_CAMERA__CONTROL_HPP_
#define V4L2_CAMERA__CONTROL_HPP_

#include <linux/videodev2.h>
#include <string>
#include <map>

namespace v4l2_camera
{

/// Type of camera control
enum class ControlType : unsigned
{
  INT        = 1,
  BOOL       = 2,
  MENU       = 3,
  BUTTON     = 4,
  INT64      = 5,
  CTRL_CLASS = 6,
  STRING     = 7,
  BITMASK    = 8
};

struct Control
{
  /// Identifies the control, set by the application
  unsigned id;

  /// Human readable name
  std::string name;

  /// Type of control
  ControlType type;

  /// Minimum value, inclusive
  int minimum;

  /// Maximum value, inclusive
  int maximum;

  /// The default value of of an integer, boolean, bitmask, menu or integer menu control
  int defaultValue;

  /// Menu item names by index. Empty if this is not a menu control
  std::map<int, std::string> menuItems;

  /// Whether the control is disabled, meaning it should be ignored
  bool disabled;

  /// Whether the control is set to inactive, e.g. when it is automatically controlled
  bool inactive;
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__CONTROL_HPP_
