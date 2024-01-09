cmake_minimum_required(VERSION 3.16)
project(SSSP)

set(CMAKE_CXX_STANDARD 17)
set(YAML_CPP_INCLUDE_DIR "/opt/homebrew/Cellar/yaml-cpp/0.8.0/include")
set(YAML_CPP_LIBRARIES "yaml-cpp")

include_directories("include")

find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

file(GLOB SOURCES "src/*.cpp")

add_executable(SSSP ${SOURCES})
target_link_libraries(SSSP ${Boost_LIBRARIES})
target_link_libraries(SSSP yaml-cpp)
