cmake_minimum_required(VERSION 3.5)
project(perception)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

find_package(PCL 1.3 REQUIRED COMPONENTS common)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create new target for planar_cluster
add_library(planar_cluster src/planar_cluster.cpp)
ament_target_dependencies(planar_cluster rclcpp rclcpp_components sensor_msgs)
target_link_libraries(planar_cluster ${PCL_LIBRARIES})

# Create new target for color_cluster
add_library(color_cluster src/color_cluster.cpp)
ament_target_dependencies(color_cluster rclcpp rclcpp_components sensor_msgs)
target_link_libraries(color_cluster ${PCL_LIBRARIES})

# Create new target for landmark_graph
add_library(landmark_graph src/landmark_graph.cpp)
ament_target_dependencies(landmark_graph rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs)
target_link_libraries(landmark_graph ${PCL_LIBRARIES})

# Create new target for border_finder
add_library(border_finder src/border_finder.cpp)
ament_target_dependencies(border_finder rclcpp rclcpp_components cv_bridge geometry_msgs nav_msgs)
target_link_libraries(border_finder ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS
  color_cluster planar_cluster landmark_graph border_finder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
