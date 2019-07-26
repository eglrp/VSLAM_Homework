CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#SET(BUILD_TYPE Release)
SET(BUILD_TYPE Debug)

PROJECT(slam_6)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

INCLUDE_DIRECTORIES("/usr/local/eigen3"
        ${OpenCV_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS})

ADD_EXECUTABLE(optical_flow optical_flow.cpp)

TARGET_LINK_LIBRARIES(optical_flow
        ${OpenCV_LIBS})



ADD_EXECUTABLE(direct_method direct_method.cpp)
TARGET_LINK_LIBRARIES(direct_method
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES})

ADD_EXECUTABLE(compute_disparity compute_disparity.cpp)
TARGET_LINK_LIBRARIES(compute_disparity
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES})