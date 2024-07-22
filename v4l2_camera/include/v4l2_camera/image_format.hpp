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

#ifndef V4L2_CAMERA__IMAGE_FORMAT_HPP_
#define V4L2_CAMERA__IMAGE_FORMAT_HPP_

#include <linux/videodev2.h>
#include <string>

namespace v4l2_camera
{

/** Image format description
 *
 * Describes what image data means
 */
struct ImageFormat
{
  explicit ImageFormat(v4l2_fmtdesc const & fd)
  : index(fd.index),
    type(fd.type),
    flags(fd.flags),
    description((const char *)fd.description),
    pixelFormat(fd.pixelformat)
  {}

  /// Number of the format in the enumeration, set by the application
  unsigned index;

  /// Type of the data stream, set by the application, probably to V4L2_BUF_TYPE_VIDEO_CAPTURE
  unsigned type;

  /// Image format description flags. Options:
  /// V4L2_FMT_FLAG_COMPRESSED and/or V4L2_FMT_FLAG_EMULATED
  unsigned flags;

  /// Human readable description of the format
  std::string description;

  /// The image format identifier as computed by the v4l2_fourcc() macro
  unsigned pixelFormat;
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__IMAGE_FORMAT_HPP_
