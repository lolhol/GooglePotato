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

file(GLOB_RECURSE GOOGLE_POTATO_SRC "src/*.cpp" "port/cpp/*.cpp")
add_library(google_potato SHARED ${GOOGLE_POTATO_SRC})

target_link_libraries(google_potato ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} cartographer)
get_target_property(INCLUDE_CARTO cartographer INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(google_potato PUBLIC ${INCLUDE_CARTO})

install(TARGETS google_potato DESTINATION /usr/lib/jni)

# Java compilation

add_custom_target(JavaCompile
        COMMAND ${CMAKE_COMMAND} -E make_directory ./java-build
        COMMAND javac -d ./java-build ../port/pwrup/google/potato/*.java
)

add_custom_target(JavaDoc
        COMMAND ${CMAKE_COMMAND} -E make_directory ./java-doc
        COMMAND javadoc -d ./java-doc ../port/pwrup/google/potato/*.java
)

add_custom_target(java
        COMMAND find java-build/port/pwrup/google/potato -name "*.class" | xargs jar cvf ./GooglePotato.jar
)


add_dependencies(java JavaCompile JavaDoc)
add_dependencies(JavaCompile google_potato)
add_dependencies(JavaDoc JavaCompile)

add_dependencies(google_potato ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cartographer)
