cmake_minimum_required(VERSION 3.5)
project(raspicam2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(class_loader REQUIRED)

FIND_LIBRARY(mmal_LIBS REQUIRED NAMES mmal PATHS /opt/vc/lib)
FIND_LIBRARY(mmalcore_LIBS REQUIRED NAMES mmal_core PATHS /opt/vc/lib)
FIND_LIBRARY(mmalutil_LIBS REQUIRED NAMES mmal_util PATHS /opt/vc/lib)
FIND_LIBRARY(mmalvcclient_LIBS REQUIRED NAMES mmal_vc_client PATHS /opt/vc/lib)
FIND_LIBRARY(vcos_LIBS REQUIRED NAMES vcos PATHS /opt/vc/lib)
FIND_LIBRARY(bcmhost_LIBS REQUIRED NAMES bcm_host PATHS /opt/vc/lib)

include_directories(include /opt/vc/include)

add_library(raspicam_common STATIC
    src/RaspiCamControl.c
    src/RaspiCLI.c
    src/RaspiPreview.c
    src/RaspiCommonSettings.c
    src/RaspiHelpers.c
    src/RaspiGPS.c
    src/libgps_loader.c
)

target_link_libraries(raspicam_common ${bcmhost_LIBS} ${vcos_LIBS} ${mmal_LIBS} ${mmalcore_LIBS} ${mmalutil_LIBS} ${mmalvcclient_LIBS})

add_library(raspicam SHARED src/raspicam.cpp)
target_link_libraries(raspicam raspicam_common)

# composable ros node
add_library(RasPiCamPublisherNode SHARED src/RasPiCamPublisherNode.cpp)
ament_target_dependencies(RasPiCamPublisherNode rclcpp rclcpp_components sensor_msgs class_loader)
target_link_libraries(RasPiCamPublisherNode raspicam)
rclcpp_components_register_nodes(RasPiCamPublisherNode "RasPiCamPublisher")

add_executable(raspicam2_node src/raspicam2_node.cpp)
target_link_libraries(raspicam2_node RasPiCamPublisherNode)

install(TARGETS
    RasPiCamPublisherNode raspicam
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS
    raspicam2_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)

ament_package()
