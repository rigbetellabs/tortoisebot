^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package v4l2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.2 (2023-05-02)
------------------
* Don't re-queue buffer before getting the data
* Break out parameter handler into separate class, reducing unnecessary startup warnings.
* Add UYVY pixel format
* Contributors: Martin Fraunhofer, Sander G. van Dijk

0.6.0 (2022-09-04)
------------------
* Use cv_bridge to perform conversions
* Fix: Image topic should be image_raw, not raw_image, in README.md
* Fix: Properly declare camera_info_url parameter
* Contributors: Marcus M. Scheunemann, Sander G. van Dijk, ijnek

0.5.0 (2022-08-06)
------------------
* Add time_per_frame parameter to adjust frame rate
* Fix: use 64 bit integers to store control valueas and ranges
* Contributors: Marcus M. Scheunemann, Sander G. van Dijk

0.4.0 (2021-01-29)
------------------
* Read-only parameters as now properly marked as such
* Use C-style strings for logging macros to adapt to new formatting constraints
  See: https://index.ros.org/doc/ros2/Releases/Release-Galactic-Geochelone/#change-in-rclcpp-s-logging-macros
* Add support monochrome cameras
* Contributors: Chris Lalancette, Sander G. van Dijk, nfry321

0.3.1 (2020-11-07)
------------------
* Handle non discrete frame sizes; fixes support for Raspberry Pi + Camera Module
* Contributors: Sander G. van Dijk

0.3.0 (2020-09-26)
------------------
* Publishing is done on private topics to enable remapping of the namespace
* CameraInfo is published in intra-process communication mode
* Added parameter descriptions
* Contributors: Christian Rauch, Marcus M. Scheunemann, Sander G. van Dijk

0.2.1 (2020-08-06)
------------------
* Hold reference to parameters callback handle
* Contributors: Jacob Perron

0.2.0 (2020-06-13)
------------------
* Set frame_id on published images, default to "camera"
* Output FOURCC code of available formats
* Add parameter for setting the pixel format
  Default to using YUYV
* Contributors: Sander G. van Dijk

0.1.1 (2019-08-12)
------------------
* Add missing rclcpp_components build dependency
* Contributors: Sander G. van Dijk

0.1.0 (2019-08-11)
------------------
