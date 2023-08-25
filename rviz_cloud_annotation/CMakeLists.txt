cmake_minimum_required(VERSION 3.22.0)
project(rviz_cloud_annotation)

find_package(ament_cmake REQUIRED)

set(ROS2_DEPENDENCIES
  rclcpp
  rviz_common 
  interactive_markers
  pcl_conversions
  tf2_ros
  tf2_eigen
  visualization_msgs
  geometry_msgs
  std_msgs
  rosidl_default_generators
  rviz_default_plugins
  rviz_ogre_vendor
  pluginlib
  )

foreach (ROS2_PACKAGE ${ROS2_DEPENDENCIES})
  find_package(${ROS2_PACKAGE} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Qt5 REQUIRED COMPONENTS Widgets)
include_directories(${Qt5_INCLUDE_DIRS})
set(CMAKE_AUTOMOC ON)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/UndoRedoState.msg
  msg/RectangleSelectionViewport.msg
  DEPENDENCIES
  geometry_msgs std_msgs 
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

###################################
## catkin specific configuration ##
###################################

###########
## Build ##
###########

add_subdirectory(src/rviz_cloud_annotation_points)

### MAIN NODE ###
add_executable(rviz_cloud_annotation_node
  src/rviz_cloud_annotation.cpp
  src/rviz_cloud_annotation_class.cpp
  )

ament_target_dependencies(rviz_cloud_annotation_node ${ROS2_DEPENDENCIES})
add_dependencies(rviz_cloud_annotation_node ${PROJECT_NAME})
target_link_libraries(rviz_cloud_annotation_node "${cpp_typesupport_target}")
target_link_libraries(rviz_cloud_annotation_node 
  rviz_cloud_annotation_points
)

target_link_libraries(rviz_cloud_annotation_node
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  )

add_dependencies(rviz_cloud_annotation_node ${PROJECT_NAME})
target_link_libraries(rviz_cloud_annotation_node "${cpp_typesupport_target}")

### RVIZ PLUGIN ###
add_library(rviz_cloud_annotation_plugin SHARED
  src/rviz_cloud_annotation_plugin.cpp
  src/rviz_select_tool.cpp
)

ament_target_dependencies(rviz_cloud_annotation_plugin ${ROS2_DEPENDENCIES})

target_link_libraries(rviz_cloud_annotation_plugin
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${Qt5_LIBRARIES}
)

add_dependencies(rviz_cloud_annotation_plugin ${PROJECT_NAME})
target_link_libraries(rviz_cloud_annotation_plugin "${cpp_typesupport_target}")

#############
## Install ##
#############

install(
  TARGETS rviz_cloud_annotation_plugin rviz_cloud_annotation_node rviz_cloud_annotation_points
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

pluginlib_export_plugin_description_file(rviz_common "rviz_plugin/plugin_description.xml")

ament_export_dependencies(
  ${ROS2_DEPENDENCIES} Eigen3 PCL Qt5
)

ament_export_libraries(
  rviz_cloud_annotation_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
