cmake_minimum_required(VERSION 2.4)
set(execName "dualSA")
project(${execName})

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

add_subdirectory(src)
add_executable(${execName} "test.cpp")
add_definitions(-std=c++11 -g -O3)

set(userlib planning)
target_link_libraries(${execName}
                        ${ZED_LIBRARIES}
                        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED}
						${userlib})
