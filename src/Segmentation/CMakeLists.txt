cmake_minimum_required(VERSION 2.8)
project( segmentation )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB seg_src ./Segmenter.cpp)
file(GLOB seg_head Segmenter.h)
file(GLOB seg_test ./main.cpp)

add_library(seg_shared  SHARED ${seg_src})
add_library(seg_static STATIC ${seg_src})

set_target_properties(seg_shared PROPERTIES OUTPUT_NAME "seg")
set_target_properties(seg_static PROPERTIES OUTPUT_NAME "seg")

INSTALL(TARGETS seg_shared seg_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

INSTALL(FILES ${seg_head}  DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

add_executable( seg_test ${seg_test} )
target_link_libraries( seg_test ${OpenCV_LIBS} )
target_link_libraries( seg_test seg_shared )



