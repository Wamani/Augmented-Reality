include_directories(
        ${PROJECT_SOURCE_DIR}/include
)


set(SOURCE_FILES
        gtest_main.cc
        ${PROJECT_SOURCE_DIR}/src/OpenGLModule.cpp
        )

add_executable(unit_test ${SOURCE_FILES})

if (${PLATFORM_WIN})
     set(CMAKE_CXX_FLAGS_RELEASE "/MT")
     set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
     target_link_libraries(unit_test debug gtestd)
     target_link_libraries(unit_test optimized gtest)
else()
     target_link_libraries(unit_test ${OpenCV_LIBS})
endif()
