cmake_minimum_required (VERSION 3.9)
project (CS6610)

include_directories(src)
include_directories(include)

include(cmake/glm.cmake.in)
include(cmake/freeglut.cmake.in)

add_executable(
    CS6610 
    src/main.cpp
    src/app.cpp
)

message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    message(STATUS "Clang detected, adding compile flags")
    target_compile_options(CS6610 PRIVATE -Wall -Wno-c++98-compat -Wno-reserved-id-macro)
    target_compile_features(CS6610 PRIVATE cxx_std_17)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "GCC detected, adding compile flags")
    target_compile_options(CS6610 PRIVATE -Wall)
    target_compile_features(CS6610 PRIVATE cxx_std_17)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    message(STATUS "msvc detected, adding compile flags")
    target_compile_options(CS6610 PRIVATE  /std:c++latest /EHsc)
else()
    message(STATUS "Could not determine compiler: ${CMAKE_CXX_COMPILER_ID}" )
    target_compile_features(CS6610 PRIVATE cxx_std_17)
    target_compile_options(CS6610 PRIVATE -Wall)
endif()

target_link_libraries(CS6610 freeglut glm)
