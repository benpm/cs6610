cmake_minimum_required (VERSION 3.9)
project (CS6610)

include_directories(src)
include_directories(include)

include(cmake/eigen.cmake.in)
include(cmake/glfw.cmake.in)
include(cmake/spdlog.cmake.in)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

add_executable(
    app 
    src/main.cpp
    src/app.cpp
    src/camera.cpp
    src/extmath.cpp
    src/glad.c
)

message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    message(STATUS "Clang detected, adding compile flags")
    target_compile_options(app PRIVATE -Wall -Wno-c++98-compat -Wno-reserved-id-macro)
    target_compile_features(app PRIVATE cxx_std_17)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "GCC detected, adding compile flags")
    target_compile_options(app PRIVATE -Wall)
    target_compile_features(app PRIVATE cxx_std_17)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    message(STATUS "msvc detected, adding compile flags")
    target_compile_options(app PRIVATE  /std:c++latest /EHsc)
else()
    message(STATUS "Could not determine compiler: ${CMAKE_CXX_COMPILER_ID}" )
    target_compile_features(app PRIVATE cxx_std_17)
    target_compile_options(app PRIVATE -Wall)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

target_link_libraries(app glfw ${OPENGL_LIBRARIES} spdlog)
