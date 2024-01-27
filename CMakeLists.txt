# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32sensors)

idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
