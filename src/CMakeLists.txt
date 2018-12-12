cmake_minimum_required(VERSION 2.4)

add_library(planning planning.cpp)
target_include_directories(planning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})