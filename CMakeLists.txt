cmake_minimum_required(VERSION 2.8.3)
project(ds_robetarme)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  mathlib
  sgaussians
  lpvDS
  lagsDS
  dynamic_reconfigure
  gaussian_process_regression
  # Eigen3
  cFunc
  utils
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

 generate_dynamic_reconfigure_options(
  cfg/lagsDS_params.cfg
)  


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
 INCLUDE_DIRS include/coupled_DS include/learned_DS
 LIBRARIES ds_robetarme
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs mathlib sgaussians lpvDS lagsDS dynamic_reconfigure cFunc utils
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES 
          #--- for coupled_DS
            ${INCLUDE_DIR}/coupled_DS/linearDS.h
            ${INCLUDE_DIR}/coupled_DS/coupledDS.h
          #--- for learned model
            )

## Specify locations of header files
## Your package locations should be listed before other locations
include_directories(include/coupled_DS include/learned_DS ${catkin_INCLUDE_DIRS})
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

## Declare a C++ executable
set(SRCS
    #--- for test
      src/coupled_DS/linearDS.cpp
      src/coupled_DS/coupledDS.cpp
    #--- for learned model
    )

add_library(ds_robetarme ${H_FILES} ${SRCS})
target_link_libraries(ds_robetarme ${catkin_LIBRARIES})



add_executable(linearDS_node src/linearDS_node.cpp)
target_link_libraries(linearDS_node ds_robetarme online_ESN cFunc limbo boost_system boost_serialization boost_filesystem)

add_executable(coupledDS_node src/coupledDS_node.cpp)
target_link_libraries(coupledDS_node ds_robetarme online_ESN cFunc limbo boost_system boost_serialization boost_filesystem utils)

### ADD Nodes that use the library

add_dependencies(linearDS_node 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)

add_dependencies(coupledDS_node 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)


target_link_libraries(linearDS_node ds_robetarme ${catkin_LIBRARIES} )
target_link_libraries(coupledDS_node ds_robetarme ${catkin_LIBRARIES} )


