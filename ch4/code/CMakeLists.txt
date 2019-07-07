cmake_minimum_required( VERSION 2.8 )
project( code )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
# 添加头文件

find_package(Pangolin REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${Pangolin_INCLUDE_DIRS})


add_executable( undistort_image undistort_image.cpp )
# 链接OpenCV库
target_link_libraries( undistort_image ${OpenCV_LIBS} )

add_executable(disparity disparity.cpp)
target_link_libraries( disparity ${Pangolin_LIBRARIES} ${OpenCV_LIBS} )


add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries( gaussnewton ${OpenCV_LIBS} )