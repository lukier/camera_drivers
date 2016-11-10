include(FindPackageHandleStandardArgs)

find_path(LIBREALSENSE_INCLUDE_DIR librealsense/rs.hpp PATHS /usr/include)
find_library(LIBREALSENSE_LIBRARY NAMES realsense PATHS /usr/lib /usr/lib32 /usr/lib64)

find_package_handle_standard_args(librealsense "Could NOT find librealsense." LIBREALSENSE_LIBRARY LIBREALSENSE_INCLUDE_DIR)

if(LIBREALSENSE_FOUND)
    find_package_message(LIBREALSENSE_FOUND "Found librealsense ${LIBREALSENSE_LIBRARY}" "[${LIBREALSENSE_LIBRARY}][${LIBREALSENSE_INCLUDE_DIR}]")
endif(LIBREALSENSE_FOUND)

mark_as_advanced(LIBREALSENSE_INCLUDE_DIR LIBREALSENSE_LIBRARY)
