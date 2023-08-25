cmake_minimum_required(VERSION 3.22.0)
project(rviz_3d_view_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
set(ROS2_DEPENDENCIES
  rclcpp 
  rviz_common
  pluginlib
  rviz_rendering
  rviz_ogre_vendor
  )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Qt5 REQUIRED COMPONENTS Widgets)
include_directories(${Qt5_INCLUDE_DIRS})

#find_package(Qt5Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

###########
## Build ##
###########

add_library(rviz_3d_view_controller SHARED
  src/rviz_orbit3d_view_controller.cpp
)

ament_target_dependencies(rviz_3d_view_controller ${ROS2_DEPENDENCIES})

target_link_libraries(rviz_3d_view_controller
  ${Eigen3_LIBRARIES}
  ${Qt5_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common "rviz_plugin/plugin_description.xml")

#############
## Install ##
#############

install(
  TARGETS rviz_3d_view_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  rviz_3d_view_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()

