cmake_minimum_required(VERSION 3.0.2)
project(tool_gravity_compensation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  pluginlib
  rviz
  sriforcesensor
  tf
)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

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
add_message_files(
  FILES
  PayloadResult.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ComputePayload.srv
  SetSamplingConfig.srv
  SetSensorMount.srv
  StepControl.srv
  tool_gravity.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  CollectLoadProfile.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
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
  INCLUDE_DIRS include
  LIBRARIES payload_identifier tool_gravity_panel
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime pluginlib roscpp rospy rviz sriforcesensor std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tool_gravity_compensation.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_library(payload_identifier src/payload_identifier.cpp)
add_dependencies(payload_identifier ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(collect_load_profile_server src/collect_load_profile_server.cpp)
add_dependencies(collect_load_profile_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(collect_load_profile_server
  payload_identifier
  ${catkin_LIBRARIES}
)

add_library(tool_gravity_panel src/tool_gravity_panel.cpp)
add_dependencies(tool_gravity_panel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tool_gravity_panel
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

add_executable(tool_gravity src/tool_gravity.cpp)
add_dependencies(tool_gravity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tool_gravity
  ${catkin_LIBRARIES}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
if(CATKIN_ENABLE_TESTING)
  catkin_install_python(PROGRAMS
    test/test_sensor_mount_tf_manager.py
    test/test_collect_load_profile_fail_closed.py
    test/test_collect_load_profile_positive.py
    test/test_payload_dataset_replay.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  catkin_add_nosetests(test/test_offline_wrench_simulator.py)
  catkin_add_nosetests(test/test_sensor_mount_tf_manager.py)
  catkin_add_nosetests(test/test_tool_gravity_panel_mode_gating.py)
  catkin_add_nosetests(test/test_payload_comparison_report.py)
  catkin_add_nosetests(test/test_payload_dataset_validation_runner.py)

  catkin_add_gtest(test_payload_identifier test/test_payload_identifier.cpp)
  if(TARGET test_payload_identifier)
    target_link_libraries(test_payload_identifier payload_identifier ${catkin_LIBRARIES} gtest_main)
  endif()
  add_rostest(test/collect_load_profile_fail_closed.test
    DEPENDENCIES collect_load_profile_server
  )
  add_rostest(test/collect_load_profile_positive.test
    DEPENDENCIES collect_load_profile_server
  )
  add_rostest(test/payload_dataset_replay.test
    DEPENDENCIES collect_load_profile_server
  )
  add_custom_target(payload_dataset_validation_report
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_payload_dataset_validation.py
            --report-dir ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}
    DEPENDS collect_load_profile_server
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    USES_TERMINAL
    COMMENT "Run payload dataset replay and print theory-vs-actual comparison report"
  )
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/offline_wrench_simulator.py
  scripts/run_payload_dataset_validation.py
  scripts/sensor_mount_tf_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS collect_load_profile_server payload_identifier tool_gravity tool_gravity_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tool_gravity_compensation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
