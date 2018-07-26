cmake_minimum_required(VERSION 2.8)

project(opencv_opengl_demo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

message("config types: ${CMAKE_CONFIGURATION_TYPES}")

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
        include
        src
)

set(SOURCE_FILES
        src/OpenGLModule.cpp
        include/opencv_opengl_demo/OpenGLModule.h
        )

add_library(opencv_opengl_demo SHARED ${SOURCE_FILES})

add_subdirectory(test)

set(OUTPUT_ROOT ${PROJECT_SOURCE_DIR}/output)

set(MY_OUTPUT_ROOT ${OUTPUT_ROOT}/opencv_opengl_demo)
install(TARGETS opencv_opengl_demo
    DESTINATION ${MY_OUTPUT_ROOT}/lib)
install(FILES ${PROJECT_SOURCE_DIR}/include/opencv_opengl_demo/OpenGLModule.h
    DESTINATION ${MY_OUTPUT_ROOT}/include/opencv_opengl_demo/)
