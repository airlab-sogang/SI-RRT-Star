cmake_minimum_required(VERSION 3.16)
project(SI_CBS)

set(CMAKE_CXX_STANDARD 17)

include_directories("include")

find_package(Boost REQUIRED)

set(YAML_CPP_INCLUDE_DIR "/opt/homebrew/Cellar/yaml-cpp/0.8.0/include")
set(YAML_CPP_LIBRARIES "/opt/homebrew/Cellar/yaml-cpp/0.8.0/lib/libyaml-cpp.dylib")

#find_package(yaml-cpp REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/Cellar/yaml-cpp/0.8.0/lib/cmake/yaml-cpp")

file(GLOB SOURCES "src/*.cpp")

add_executable(SI-CBS ${SOURCES})

target_link_libraries(SI-CBS ${Boost_LIBRARIES})
target_link_libraries(SI-CBS ${YAML_CPP_LIBRARIES})
