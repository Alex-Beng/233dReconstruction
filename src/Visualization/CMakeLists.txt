cmake_minimum_required(VERSION 2.8)
project( visualization )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


file(GLOB cam_rt ./CameraRt.cpp)
file(GLOB _test ./test.cpp)

add_executable( cam_rt ${cam_rt} )
target_link_libraries( cam_rt ${OpenCV_LIBS} )

add_executable( _test ${_test} )
target_link_libraries( _test ${OpenCV_LIBS} )