cmake_minimum_required(VERSION 2.8)

project(direct-ba)
set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-O3" )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Cholmod REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
        g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
        g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/ceres)
include_directories(
        ${Pangolin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${OpenCV_DIRS}
        ${Boost_INCLUDE_DIRS})
message("find g2o libraries in: " ${G2O_INCLUDE_DIR})
message("find g2o lib in " ${G2O_LIBS})
message("find cholmod in " ${CHOLMOD_INCLUDE_DIR})

add_executable(${PROJECT_NAME} directBA.cpp)

target_link_libraries(${PROJECT_NAME}
        ${G2O_LIBS}
        ${CHOLMOD_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBS})
