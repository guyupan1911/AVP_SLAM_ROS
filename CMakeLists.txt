cmake_minimum_required(VERSION 3.0.2)
project(avp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
# add_definitions("-Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
  SaveMap.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES AVP_Simulator
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/src/GridMap"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/mapping"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Simulator"
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/AVP_Simulator.cpp
# )

## Add cmake target dependencies of the library~
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/AVP_Simulator_node.cpp)

#add_executable(simulator_node src/simulator_node_main.cc 
#            src/Simulator/simulating_node.cc 
#            src/Simulator/odometry_simulator.cc 
#            src/Simulator/hdMap.cc
#            )
#add_executable(keyBoardCtl_node src/keyBoardCtl_node.cc)
add_executable(mapping_node src/mapping_node_main.cc
            src/mapping/local_trajectory_builder.cc
            src/mapping/pose_extrapolator.cc
            src/mapping/transform.cc
            src/mapping/motion_filter.cc
            src/mapping/global_trajectory_builder.cc
            src/GridMap/submap.cc
            src/mapping/pose_graph.cc
            src/mapping/constraint.cc
            src/mapping/optimization_problem.cc
            src/mapping/spa_cost_function.cc
            src/mapping/real_time_correlative_scan_matcher.cc
            src/GridMap/gridmap.cc
            src/GridMap/probability_values.cc
            src/GridMap/probability_grid_range_data_inserter.cc
            src/GridMap/value_conversion_tables.cc
            src/mapping/correlative_scan_matcher.cc
            src/mapping/occupied_space_cost_function.cc
            src/mapping/ceres_scan_matcher_2d.cc
            src/mapping/fast_correlative_scan_matcher.cc
            src/mapping/io.cc
)

add_executable(fast_CSM_test_node src/fast_CSM_test.cc
            src/mapping/local_trajectory_builder.cc
            src/mapping/pose_extrapolator.cc
            src/mapping/transform.cc
            src/mapping/motion_filter.cc
            src/mapping/global_trajectory_builder.cc
            src/GridMap/submap.cc
            src/mapping/pose_graph.cc
            src/mapping/constraint.cc
            src/mapping/optimization_problem.cc
            src/mapping/spa_cost_function.cc
            src/mapping/real_time_correlative_scan_matcher.cc
            src/GridMap/gridmap.cc
            src/GridMap/probability_values.cc
            src/GridMap/probability_grid_range_data_inserter.cc
            src/GridMap/value_conversion_tables.cc
            src/mapping/correlative_scan_matcher.cc
            src/mapping/occupied_space_cost_function.cc
            src/mapping/ceres_scan_matcher_2d.cc
            src/mapping/fast_correlative_scan_matcher.cc
            src/mapping/io.cc
)

add_executable(pose_adjustment_node src/pose_adjustment.cc)
target_link_libraries(pose_adjustment_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

#target_link_libraries(simulator_node
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}
#  )

#target_link_libraries(keyBoardCtl_node
#${catkin_LIBRARIES}
#)

target_link_libraries(mapping_node   
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES}
)

target_link_libraries(fast_CSM_test_node   
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES}
)

# add_executable(add_two_ints_server src/add_two_ints_server.cpp)
# target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
# add_dependencies(add_two_ints_server avp_gencpp)

# add_executable(add_two_ints_client src/add_two_ints_client.cpp)
# target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
# add_dependencies(add_two_ints_client avp_gencpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_AVP_Simulator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
