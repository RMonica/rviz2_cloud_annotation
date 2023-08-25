cmake_minimum_required(VERSION 3.22.0)
project(rviz_cloud_annotation_points)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

### LIBRARY ###
add_library(rviz_cloud_annotation_points SHARED
  point_neighborhood.cpp         
  rviz_cloud_annotation_c_interface.cpp  
  rviz_cloud_annotation_points.cpp     
  point_neighborhood_search.cpp  
  rviz_cloud_annotation_point_plane.cpp
  rviz_cloud_annotation_points_io.cpp
  rviz_cloud_annotation_undo.cpp
)

### MAIN EXECUTABLE ###
add_executable(rviz_cloud_annotation_test
  rviz_cloud_annotation_test.cpp
)

target_link_libraries(rviz_cloud_annotation_test
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  rviz_cloud_annotation_points
)

install(
  TARGETS rviz_cloud_annotation_points rviz_cloud_annotation_test
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/rviz_cloud_annotation
)
