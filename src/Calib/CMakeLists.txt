cmake_minimum_required(VERSION 2.8)
project( calib )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB charuco_cali ./calibrate_camera_charuco.cpp)


add_executable( charuco_cali ${charuco_cali} )
target_link_libraries( charuco_cali ${OpenCV_LIBS} )

