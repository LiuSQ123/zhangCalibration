cmake_minimum_required(VERSION 3.12)
project(zhang_calibration)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE DEBUG)
#暂且设置为 release 模式
#set(CMAKE_BUILD_TYPE "Release")
# DEBUG 编译FLAG
ADD_DEFINITIONS(-DDEBUG_FLAG)

find_package(Eigen3 REQUIRED)

set(OpenCV_DIR /home/liushiqi/opencv/build)
set(OpenCV_INCLUDE_DIRS /home/liushiqi/opencv/include)
#set(OpenCV_LIBRARIES /home/liushiqi/opencv/build/build/lib)
set(OpenCV_LIBRARIES /home/liushiqi/opencv/build/lib)

find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

#设定库输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#输出库
ADD_LIBRARY(CALIB src/ZZYCalibration.cpp)

add_executable(zhang_calibration main.cpp)
target_link_libraries(zhang_calibration CALIB ${OpenCV_LIBS} )

add_executable(openCVCalibration openCVCalibSample.cpp)
target_link_libraries(openCVCalibration  ${OpenCV_LIBS} )