cmake_minimum_required(VERSION 3.16)

# ESP-IDF imported targets only define IMPORTED_LOCATION for config "None".
# CLion injects -DCMAKE_BUILD_TYPE=Release which triggers CMP0111.
# ESP-IDF manages its own optimization levels via sdkconfig, not CMake build types.
if(POLICY CMP0111)
    cmake_policy(SET CMP0111 OLD)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(shelfbot)