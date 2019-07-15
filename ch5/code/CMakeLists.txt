cmake_minimum_required( VERSION 2.8 )
project( vo1 )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# find_package( OpenCV 3.1 REQUIRED )
find_package( OpenCV REQUIRED ) # use this if in OpenCV2
FIND_PACKAGE(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${Sophus_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${Pangolin_INCLUDE_DIRS}
)

add_executable( computeORB computeORB.cpp  )
target_link_libraries( computeORB ${OpenCV_LIBS} )

add_executable( E2Rt E2Rt.cpp  )
target_link_libraries( E2Rt ${Sophus_LIBRARIES} )

add_executable( GN-BA GN-BA.cpp  )
target_link_libraries( GN-BA ${Sophus_LIBRARIES} )

add_executable(ICP ICP.cpp)
target_link_libraries(ICP ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})