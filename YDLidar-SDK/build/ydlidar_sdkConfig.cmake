SET( YDLIDAR_SDK_LIBRARIES  "ydlidar_sdk;pthread;rt" CACHE INTERNAL "YDLIDAR_SDK libraries" FORCE )
SET( YDLIDAR_SDK_INCLUDE_DIRS  /usr/local/include/src;/usr/local/include CACHE INTERNAL "YDLIDAR_SDK include directories" FORCE )
SET( YDLIDAR_SDK_LIBRARY_DIRS  CACHE INTERNAL "YDLIDAR_SDK library directories" FORCE )

mark_as_advanced( YDLIDAR_SDK_LIBRARIES )
mark_as_advanced( YDLIDAR_SDK_LIBRARY_DIRS )
mark_as_advanced( YDLIDAR_SDK_INCLUDE_DIRS )



# Compute paths
get_filename_component( PACKAGE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )

# This file, when used for INSTALLED code, does not use Targets... sigh.
## Library dependencies (contains definitions for IMPORTED targets)
#if(NOT TARGET "YDLIDAR_SDK_LIBRARIES" AND NOT "YDLIDAR_SDK_BINARY_DIR")
#    include( "${PACKAGE_CMAKE_DIR}/YDLIDAR_SDKTargets.cmake" )
#    include( "${PACKAGE_CMAKE_DIR}/YDLIDAR_SDKConfigVersion.cmake" )
#endif()

#SET(YDLIDAR_SDK_LIBRARIES )
#SET(YDLIDAR_SDK_LIBRARY )
#SET(YDLIDAR_SDK_INCLUDE_DIRS /usr/local/include/src;/usr/local/include)
#SET(YDLIDAR_SDK_LINK_DIRS )
