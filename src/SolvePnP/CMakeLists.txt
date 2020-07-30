cmake_minimum_required(VERSION 2.8)
project( solve-pnp )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


file(GLOB pnp_src ./PnpSolver.cpp)
file(GLOB pnp_head ./PnpSolver.h)
file(GLOB pnp_test ./main.cpp)

# lib
add_library(pnp_shared  SHARED ${pnp_src})
add_library(pnp_static STATIC ${pnp_src})


set_target_properties(pnp_shared PROPERTIES OUTPUT_NAME "pnp")
set_target_properties(pnp_static PROPERTIES OUTPUT_NAME "pnp")

# install 
INSTALL(TARGETS pnp_shared pnp_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

INSTALL(FILES ${pnp_head}  DESTINATION include)

add_executable( pnp_test ${pnp_test} )
target_link_libraries( pnp_test ${OpenCV_LIBS} )



