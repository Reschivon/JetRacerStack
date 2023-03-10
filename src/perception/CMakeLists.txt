cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
	pcl_ros nodelet roscpp cv_bridge
)

find_package(PCL 1.3 REQUIRED COMPONENTS common)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS src
   LIBRARIES 
   CATKIN_DEPENDS nodelet roscpp cv_bridge
)

include_directories(include
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

# Create new target for color_cluster
add_library(color_cluster src/color_cluster.cpp)
add_dependencies(color_cluster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(color_cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Create new target for color_cluster
add_library(planar_cluster src/planar_cluster.cpp)
add_dependencies(planar_cluster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planar_cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Create new target for landmark_graph
add_library(landmark_graph src/landmark_graph.cpp)
add_dependencies(landmark_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(landmark_graph ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Create new target for border_finder
add_library(border_finder src/border_finder.cpp)
add_dependencies(border_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(border_finder ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
  planar_cluster color_cluster landmark_graph border_finder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/cone_finder.py
#   scripts/pointcloud_fix.py
#   scripts/cone_list_viz.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
