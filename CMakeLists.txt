cmake_minimum_required(VERSION 3.5)
project(optitrack_pose_broadcaster)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  mocap4r2_msgs
)
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_library(gt_pose_broadcaster SHARED
  src/gt_pose_broadcaster_node.cpp)

target_include_directories(gt_pose_broadcaster
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${rclcpp_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${mocap4r2_msgs_INCLUDE_DIRS})

ament_target_dependencies(gt_pose_broadcaster
  PUBLIC
  rclcpp
  rclcpp_components
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  mocap4r2_msgs
  )

rclcpp_components_register_node(gt_pose_broadcaster PLUGIN "optitrack_pose_broadcaster::GTPoseBroadcaster" EXECUTABLE gt_pose_broadcaster_node)
install(TARGETS gt_pose_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()