# v4l2_camera

A ROS 2 camera driver using Video4Linux2 (V4L2).

## Features

* Lists and exposes all user-settable controls of your camera as ROS 2
  parameters.
* Uses `cv_bridge` to convert raw frames to ROS 2 messages, so
  supports a wide range of encoding conversions.
* Supports `image_transport` to enable compression.
* Supports composing the camera node and using ROS 2 intra-process
  commmunication with zero-copy messaging.

## Supported Cameras

The package should work with any camera that properly supports V4L2,
which for instance should include most USB cameras. A good way to
check your camera is to install the `v4l2-utils` package (on Debian
based systems), and run the `v4l2-compliance` tool. If that reports
no, or limited amounts of, failures or warnings, you should be good to
go.

The following cameras have in any case been proven to work:

| Brand        | Type                    | Driver        | Notes                                          |
|--------------|-------------------------|---------------|------------------------------------------------|
| Logitech     | C920                    | `uvcvideo`    |                                                |
| Logitech     | BRIO 4K Pro             | `uvcvideo`    |                                                |
| Microdia     | Integrated Webcam HD    | `uvcvideo`    | Integrated laptop camera, e.g. Dell XPS series |
| Raspberry Pi | Camera Module V2 (Noir) | `bm2835 mmal` | See Raspberry Pi Support notes below           |

Do let us know about other devices you successfully use this driver
for, for instance through a merge request.

### Raspberry Pi Support

Raspberry Pi has moved to a [new camera stack based on
libcamera](https://www.raspberrypi.com/news/an-open-source-camera-stack-for-raspberry-pi-using-libcamera/). As
part of this, by default the Broadcom Unicam driver will be loaded for
Raspberry Pi camera modules. This is still a V4L2 driver, however it
has a very limited scope and only provides raw Bayer images. Any
available Image Signal Processors, that can turn those raw images into
more useful formats such as RGB or JPEG, are now exposed as separate
V4L2 devices. This means that responsibility is pushed to an
application such as `v4l2_camera` to operate the multiple devices,
rather than the single all-in-one device that most other V4L2 drivers
expose. Supporting this is out of scope of this package.

You can find out whether the Unicam driver is used by installing the
`v4l-utils` package, running:
```shell
v4l2-ctl -D
```
and checking for `bcm2835-unicam`. You can also install and run the
`v4l2_camera` node, which mentions`unicam` as part of the driver name.

Luckily, for at least some devices, it is possible to load the legacy
driver instead of the new Unicam driver, by [changing settings in the
`/boot/config.txt`
file](https://www.raspberrypi.com/documentation/computers/config_txt.html):
* Set
  [`camera_autodetect=0`](https://www.raspberrypi.com/documentation/computers/config_txt.html#camera_auto_detect)
  to prevent hardware overlays that use the Unicam driver to be
  loaded.
* Set
  [`start_x=1`](https://www.raspberrypi.com/documentation/computers/config_txt.html#start_x-start_debug)
  to enable the camera using the legacy driver.

Now when you reboot and run `v4l2-ctl -D` again, or the `v4l2_camera`
node, you should see the driver being referred to as `bm2835 mmal`
instead. The `v4l2_camera` node should then operate as normal.

## Installation

### ROS package install
This package is available from the ROS package repositories and can
therefore be installed with the following command and your ROS version
name:

```shell
sudo apt-get install ros-${ROS_DISTRO}-v4l2-camera
```

### Building from source
To build this package from source, simply clone this repository into
your workspace, install dependencies, and build it using `colcon`:

```shell
git clone --branch ${ROS_DISTRO} https://gitlab.com/boldhearts/ros2_v4l2_camera.git src/v4l2_camera
rosdep install --from-paths src/v4l2_camera --ignore-src -r -y
colcon build
```

That should be sufficient in most cases, but [this
article](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304)
goes into further detail. It focuses on Raspberry Pi OS with the
Raspberry Pi Camera Module V2, but the steps should be generally
applicable.

### Basic Usage
Run the camera node to publish camera images, using the default
parameters:

```shell
ros2 run v4l2_camera v4l2_camera_node
```

You can use `rqt-image-view` to preview the images (open another terminal):

```shell
sudo apt-get install ros-${ROS_DISTRO}-rqt-image-view
ros2 run rqt_image_view rqt_image_view
```

## Nodes

### v4l2_camera_node

The `v4l2_camera_node` interfaces with standard V4L2 devices and
publishes images as `sensor_msgs/Image` messages.

#### Published Topics

* `/image_raw` - `sensor_msgs/Image`

    The image.

#### Parameters

* `video_device` - `string`, default: `"/dev/video0"`

    The device the camera is on.

* `pixel_format` - `string`, default: `"YUYV"`

    The pixel format to request from the camera. Must be a valid four
    character '[FOURCC](http://fourcc.org/)' code [supported by
    V4L2](https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/videodev.html)
    and by your camera. The node outputs the available formats
    supported by your camera when started.
    Currently supported: `"YUYV"`, `"UYVY"`, or `"GREY"`

* `output_encoding` - `string`, default: `"rgb8"`

    The encoding to use for the output image. Can be any supported by
    `cv_bridge` given the input pixel format. Currently these are:
    * `"YUYV"`: `"yuv422_yuy2"` (no conversion), or `"mono8"`, `"rgb8"`,
    `"bgr8"`, `"rgba8"` and `"bgra8"`, plus their 16 bit variants
    * `"UYVY"`: `"yuv422"` (no conversion), or `"mono8"`, `"rgb8"`,
    `"bgr8"`, `"rgba8"` and `"bgra8"`, plus their 16 bit variants
    * `"GREY"`: `"mono8"` (no conversion), `"rgb8"`,
    `"bgr8"`, `"rgba8"` and `"bgra8"`, plus their 16 bit variants

* `image_size` - `integer_array`, default: `[640, 480]`

    Width and height of the image.

* Camera Control Parameters

    Camera controls, such as brightness, contrast, white balance, etc,
    are automatically made available as parameters. The driver node
    enumerates all controls, and creates a parameter for each, with
    the corresponding value type. The parameter name is derived from
    the control name reported by the camera driver, made lower case,
    commas removed, and spaces replaced by underscores. So
    `Brightness` becomes `brightness`, and `White Balance, Automatic`
    becomes `white_balance_automatic`.

## Compressed Transport

This package uses `image_transport` to publish images and make
compression possible. However, by default it only supports raw
transfer, additional plugins are required to enable compression. These
need to be installed separately, either cloning an building [them from
source](https://github.com/ros-perception/image_transport_plugins), or
installing the ready made package:

```shell
sudo apt-get install ros-${ROS_DISTRO}-image-transport-plugins
```

Once installed, they will be automatically used by the driver and
additional topics will be available, including
`/image_raw/compressed`.
