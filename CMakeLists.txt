cmake_minimum_required(VERSION 3.0.2)
project(reachability)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  visualization_msgs
  roscpp
  std_msgs
  trac_ik_lib
  moveit_visual_tools
  message_generation
)

find_package(orocos_kdl REQUIRED)
find_package(kdl_conversions REQUIRED)
find_package(pr2_ikfast_right_arm_plugin REQUIRED)

add_service_files(
  FILES
  InvReach.srv
)

generate_messages(
 DEPENDENCIES
 geometry_msgs std_msgs
 visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
    workspace_spreader
    roscpp
    pr2_right_arm_moveit_ikfast_plugin
  CATKIN_DEPENDS
		moveit_ros_planning_interface
    visualization_msgs
    trac_ik_lib
    moveit_visual_tools
    message_runtime
	DEPENDS
    orocos_kdl
    kdl_conversions

)

include_directories(SYSTEM
	${THIS_PACKAGE_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${pr2_ikfast_right_arm_plugin_INCLUDE_DIRS})

install(
  FILES
  pr2_right_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_library(workspace_spreader  src/workspace_spreader.cpp)
target_link_libraries(workspace_spreader  ${catkin_LIBRARIES} )
add_library(base_planning  src/base_planning.cpp)
target_link_libraries(base_planning  ${catkin_LIBRARIES} )

add_executable(recorder src/recorder.cpp)
target_link_libraries(recorder
  ${catkin_LIBRARIES}
)
add_dependencies(recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(recorder workspace_spreader ${catkin_LIBRARIES})

add_executable(recorder_ikfast src/recorder_ikfast.cpp)
target_link_libraries(recorder_ikfast
  ${catkin_LIBRARIES}
)
add_dependencies(recorder_ikfast ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(recorder_ikfast workspace_spreader pr2_right_arm_moveit_ikfast_plugin ${catkin_LIBRARIES})


add_executable(ik src/ik.cpp)
target_link_libraries(ik
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_conversions_LIBRARIES}
)
add_dependencies(ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ik workspace_spreader ${catkin_LIBRARIES})

add_executable(inverse_map_build src/inverse_map_build.cpp)
target_link_libraries(inverse_map_build
  ${catkin_LIBRARIES}
)
target_link_libraries(inverse_map_build  ${catkin_LIBRARIES} )
add_dependencies(inverse_map_build ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(inverse_map_build_2 src/inverse_map_build_2.cpp)
target_link_libraries(inverse_map_build_2
  ${catkin_LIBRARIES}
)
target_link_libraries(inverse_map_build_2  ${catkin_LIBRARIES} )
add_dependencies(inverse_map_build_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(inv_server_copy src/inv_server_copy.cpp)
target_link_libraries(inv_server_copy base_planning
  ${catkin_LIBRARIES})
add_dependencies(inv_server_copy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(inv_server src/inv_server.cpp)
target_link_libraries(inv_server base_planning
  ${catkin_LIBRARIES})
add_dependencies(inv_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(inv_client src/inv_client.cpp)
target_link_libraries(inv_client
  ${catkin_LIBRARIES})

  add_executable(base_sub src/base_sub.cpp)
target_link_libraries(base_sub
  ${catkin_LIBRARIES})



add_executable(forward src/forward.cpp)
target_link_libraries(forward
  ${catkin_LIBRARIES}
)
add_dependencies(forward ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

