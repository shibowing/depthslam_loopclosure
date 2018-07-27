cmake_minimum_required(VERSION 2.8.3)
project(depthslam_loopclosure)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)
find_package(PCL  REQUIRED)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/config/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )
FIND_PACKAGE( Cholmod REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} )


#find_package( Sophus REQUIRED )
#include_directories( ${Sophus_INCLUDE_DIRS} )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs eigen_conversions
  DEPENDS eigen Boost
)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
set(THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        )
add_library( myslam SHARED
        #src/depth_map.cpp
        src/KeyframeDatabase.cpp
        src/LoopClosure.cpp
        src/TransformCalculation.cpp
        src/Matrixtransform.cpp
        src/IRONmap.cpp)
target_link_libraries( myslam
        ${THIRD_PARTY_LIBS}
        )

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#add_executable(vio_depth   src/depth_map.cpp )
add_executable(IRONmap   src/IRONmap.cpp )
target_link_libraries(IRONmap myslam)

#add_executable (ICP_refine src/ICP_refine.cpp)
#target_link_libraries(ICP_refine myslam)

#add_executable(optimize src/simple_optimize.cpp)
#target_link_libraries(optimize ${catkin_LIBRARIES} ${Boost_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} )

#target_link_libraries(loop_closure ${catkin_LIBRARIES} ${Boost_LIBRARIES} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})
# add_executable(processDepthmap src/processDepthmap.cpp)
# target_link_libraries(processDepthmap ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_executable(registerPointCloud src/registerPointCloud.cpp)
# target_link_libraries(registerPointCloud ${catkin_LIBRARIES} ${Boost_LIBRARIES})

