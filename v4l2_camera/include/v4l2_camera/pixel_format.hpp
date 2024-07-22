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

#ifndef V4L2_CAMERA__PIXEL_FORMAT_HPP_
#define V4L2_CAMERA__PIXEL_FORMAT_HPP_

#include <linux/videodev2.h>
#include <string>

namespace v4l2_camera
{

/** Format and layout of an image in memory
 *
 * Describes how data is structured
 */
struct PixelFormat
{
  PixelFormat()
  {}

  explicit PixelFormat(v4l2_pix_format const & pf)
  : width{pf.width},
    height{pf.height},
    pixelFormat{pf.pixelformat},
    bytesPerLine{pf.bytesperline},
    imageByteSize{pf.sizeimage}
  {}

  /// Image width in pixels
  unsigned width;

  /// Image height in pixels
  unsigned height;

  /// The pixel format or type of compression, set by the application
  unsigned pixelFormat;

  /// Distance in bytes between the leftmost pixels in two adjacent lines
  unsigned bytesPerLine;

  /// Size in bytes of the buffer to hold a complete image, set by the driver
  unsigned imageByteSize;
};

}  // namespace v4l2_camera

#endif  // V4L2_CAMERA__PIXEL_FORMAT_HPP_
