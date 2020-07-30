cmake_minimum_required(VERSION 2.8)
project( take-photo )

file(GLOB take_photo ./demo.cpp)

add_executable( take_photo ${take_photo} )
target_link_libraries( take_photo ${OpenCV_LIBS} )
