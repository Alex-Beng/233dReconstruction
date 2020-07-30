cmake_minimum_required(VERSION 2.8)
project( calib )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB _3cali ./3calibration.cpp)
file(GLOB charuco ./calibrate_camera_charuco.cpp)
file(GLOB cali_demo ./demo.cpp)
file(GLOB my_cali ./MyCaliOnline.cpp)

add_executable( _3cali ${_3cali} )
target_link_libraries( _3cali ${OpenCV_LIBS} )

add_executable( charuco ${charuco} )
target_link_libraries( charuco ${OpenCV_LIBS} )

add_executable( cali_demo ${cali_demo} )
target_link_libraries( cali_demo ${OpenCV_LIBS} )

add_executable( my_cali ${my_cali} )
target_link_libraries( my_cali ${OpenCV_LIBS} )
