cmake_minimum_required(VERSION 3.0.2)
project(trajectory_optimization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  octomap_ros
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(NLopt REQUIRED)
# set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})


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
 INCLUDE_DIRS include "/usr/local/include/QuadProg++" "/home/zhefan/Packages/ACADOtoolkit" "/home/zhefan/Packages/ACADOtoolkit"
 "/home/zhefan/Packages/ACADOtoolkit/acado" "/home/zhefan/Packages/ACADOtoolkit/acado/utils"
 # LIBRARIES trajectory_optimization
 # CATKIN_DEPENDS roscpp rospy std_msgs
 # DEPENDS sys/tem_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/local/include/QuadProg++"
  "/home/zhefan/Packages/ACADOtoolkit"
  "/home/zhefan/Packages/ACADOtoolkit/acado"
  "/home/zhefan/Packages/ACADOtoolkit/acado/utils"
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/trajectory_optimization.cpp
# )
add_library(polyTrajGen src/polyTrajGen.cpp)
add_library(mapModuleOctomap src/mapModuleOctomap.cpp)
add_library(timeOptimizer src/timeOptimizer.cpp)
add_library(mpcPlanner src/mpcPlanner.cpp)
add_library(mavrosTest src/mavrosTest.cpp)
add_library(fakeDetector src/fakeDetector.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/trajectory_optimization_node.cpp)
add_executable(waypoint_path_vis src/waypoint_path_vis.cpp)
add_executable(test_generate_trajectory src/test/test_generate_trajectory.cpp)
add_executable(test_quadprog src/test/test_quadprog.cpp)
add_executable(test_mapModuleOctomap src/test/test_mapModuleOctomap.cpp)
add_executable(test_timeOptimizer src/test/test_timeOptimizer.cpp)
add_executable(test_staticPlanner src/test/test_staticPlanner.cpp)
add_executable(test_acado src/test/test_acado.cpp)
add_executable(test_mpcPlanner src/test/test_mpcPlanner.cpp)
add_executable(test_gazeboTrajectory src/test/test_gazeboTrajectory.cpp)
add_executable(test_mavros_node src/test/test_mavros_node.cpp)
add_executable(test_detector src/test/test_detector.cpp)
add_executable(test_dynamicPlanner src/test/test_dynamicPlanner.cpp)
# add_executable(test_eigen src/test_eigen.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(waypoint_path_vis ${catkin_LIBRARIES})
target_link_libraries(polyTrajGen "/usr/local/lib/libquadprog.a")
target_link_libraries(timeOptimizer "/usr/local/lib/libquadprog.a")
target_link_libraries(test_generate_trajectory ${catkin_LIBRARIES} polyTrajGen)
target_link_libraries(test_quadprog ${catkin_LIBRARIES} "/usr/local/lib/libquadprog.a")
target_link_libraries(mapModuleOctomap ${catkin_LIBRARIES})
target_link_libraries(mavrosTest mpcPlanner polyTrajGen mapModuleOctomap fakeDetector)
target_link_libraries(test_mapModuleOctomap ${catkin_LIBRARIES} mapModuleOctomap polyTrajGen)
target_link_libraries(test_timeOptimizer ${catkin_LIBRARIES} mapModuleOctomap polyTrajGen timeOptimizer)
target_link_libraries(test_staticPlanner ${catkin_LIBRARIES} mapModuleOctomap polyTrajGen)
target_link_libraries(test_acado ${catkin_LIBRARIES} "/home/zhefan/Packages/ACADOtoolkit/build/lib/libacado_toolkit_s.so")
target_link_libraries(mpcPlanner "/home/zhefan/Packages/ACADOtoolkit/build/lib/libacado_toolkit_s.so")
target_link_libraries(test_mpcPlanner ${catkin_LIBRARIES} mpcPlanner polyTrajGen mapModuleOctomap)
target_link_libraries(test_gazeboTrajectory ${catkin_LIBRARIES} mpcPlanner polyTrajGen mapModuleOctomap)
target_link_libraries(test_mavros_node ${catkin_LIBRARIES} mavrosTest)
target_link_libraries(test_detector ${catkin_LIBRARIES} fakeDetector)
target_link_libraries(test_dynamicPlanner ${catkin_LIBRARIES} mpcPlanner polyTrajGen mapModuleOctomap)

# target_link_libraries(test_eigen ${catkin_LIBRARIES})

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory_optimization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
