cmake_minimum_required(VERSION 3.21)
project(RePlace)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
add_library(${PROJECT_NAME} SHARED
        include/ORBextractor.h
        include/Frame.h
        include/ORBmatcher.h
        include/MapPoint.h
        include/Optimizer.h
        include/Converter.h
        src/ORBextractor.cpp
        src/Frame.cpp
        src/ORBmatcher.cpp
        src/MapPoint.cpp
        src/Optimizer.cpp
        src/Converter.cpp)

set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

#add_subdirectory(Thirdparty/g2o)

add_executable( run_demo main.cpp )
target_link_libraries( run_demo
        ${OpenCV_LIBS}
        ${DBoW3_LIBS}
        ${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)


