cmake_minimum_required(VERSION 2.8)
project( _233drecon )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin)


find_package( OpenCV 3.0 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )


# 添加子目录
add_subdirectory(src/Calib)
add_subdirectory(src/Recon)
add_subdirectory(src/Segmentation)
add_subdirectory(src/SolvePnP)
add_subdirectory(src/Visualization)

