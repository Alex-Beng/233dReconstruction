cmake_minimum_required(VERSION 2.8)
project( recon )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB recon_src ./main.cpp)

add_executable( recon_main ${recon_src} )
target_link_libraries( recon_main seg_shared )
target_link_libraries( recon_main pnp_shared )
target_link_libraries( recon_main ${OpenCV_LIBS} )

