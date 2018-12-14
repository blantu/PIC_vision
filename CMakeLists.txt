cmake_minimum_required(VERSION 2.4)
set(execName dualSA)
project(${execName})

if (WIN32)
	MESSAGE(STATUS "Now is windows")
	SET(OpenCV_DIR "D:\CV\opencv-3.4.1\build\x64\vc15\lib")
elseif (UNIX)
	MESSAGE(STATUS "Now is UNIX-like OS's.")
	add_definitions(-Wno-format-extra-args)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
endif ()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

add_executable(${execName} main.cpp ${HDR_FILES} ${SRC_FILES})
add_definitions(-std=c++11 -g -O3)

set(userlib planning sensor)
target_link_libraries(${execName}
                        ${ZED_LIBRARIES}
						${SPECIAL_OS_LIBS}
						${OpenCV_LIBRARIES}
                        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
