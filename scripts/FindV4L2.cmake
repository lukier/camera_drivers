# - Try to find the V4L2 library
#
#  This module defines the following variables
#
#  V4L2_FOUND - Was V4L2 found
#  V4L2_INCLUDE_DIRS - the V4L2 include directories
#  V4L2_LIBRARIES - Link to this
#
#  This module accepts the following variables
#
#  V4L2_ROOT - Can be set to V4L2 install path or Windows build path
#

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_V4L2 REQUIRED libv4l2)
if(PC_V4L2_FOUND)
    set(V4L2_FOUND TRUE)
    set(V4L2_INCLUDE_DIRS ${PC_V4L2_INCLUDE_DIRS})
    set(V4L2_LIBRARY_DIRS ${PC_V4L2_LIBRARY_DIRS})
    set(V4L2_LIBRARIES ${PC_V4L2_LIBRARIES})
else()
    set(V4L2_FOUND FALSE)
endif()