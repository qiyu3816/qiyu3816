cmake_minimum_required(VERSION 2.8)

PROJECT(armordetection)
FIND_PACKAGE(OpenMP REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11")

find_package(Boost COMPONENTS filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)   
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(. DIR_SRCS)

add_executable(armordetection_result_exe ${DIR_SRCS})
target_link_libraries(armordetection_result_exe -lboost_system ${OpenCV_LIBS} ${Boost_LIBRARIES})
