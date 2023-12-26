cmake_minimum_required(VERSION 3.16)
project(SI_CBS)

set(CMAKE_CXX_STANDARD 17)

include_directories("include")

find_package(Boost REQUIRED)
#find_package(yaml-cpp REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
#include_directories(${YAML_CPP_INCLUDE_DIR})

file(GLOB SOURCES "src/*.cpp")

add_executable(SI-CBS ${SOURCES}
        include/SharedEnv.h)
target_link_libraries(SI-CBS ${Boost_LIBRARIES})
#target_link_libraries(SI-CBS yaml-cpp)
