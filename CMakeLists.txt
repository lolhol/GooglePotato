cmake_minimum_required(VERSION 3.10)

project(GooglePotato)

set(CMAKE_BUILD_TYPE Release)

find_package(cartographer REQUIRED)

find_package(Boost COMPONENTS iostreams REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()

include_directories(/usr/lib/jvm/java-1.17.0-openjdk-amd64/include /usr/lib/jvm/java-1.17.0-openjdk-amd64/include/linux)

add_compile_options(-w)
add_compile_options(-g)

file(GLOB_RECURSE GOOGLE_POTATO_SRC "src/*.cpp")
add_library(google_potato SHARED ${GOOGLE_POTATO_SRC})

target_link_libraries(google_potato ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} cartographer)
get_target_property(INCLUDE_CARTO cartographer INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(google_potato PUBLIC ${INCLUDE_CARTO})

