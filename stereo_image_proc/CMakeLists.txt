cmake_minimum_required(VERSION 3.5)

project(stereo_image_proc)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_proc REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_components
  rcutils
  class_loader
  cv_bridge
  image_geometry
  image_proc
  image_transport
  message_filters
  sensor_msgs
  stereo_msgs
)


# Disparity component
add_library(disparity SHARED
  src/disparity.cpp
  src/processor.cpp)

target_compile_definitions(disparity
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(disparity
  ${dependencies}
)

rclcpp_components_register_nodes(disparity "stereo_image_proc::DisparityNode")
set(node_plugins "${node_plugins}stereo_image_proc::DisparityNode;$<TARGET_FILE:disparity>\n")

# Point Cloud2 component
add_library(point_cloud2 SHARED
  src/point_cloud2.cpp)

target_compile_definitions(point_cloud2
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(point_cloud2
  ${dependencies}
)

rclcpp_components_register_nodes(point_cloud2 "stereo_image_proc::PointCloud2Node")
set(node_plugins "${node_plugins}stereo_image_proc::PointCloud2Node;$<TARGET_FILE:point_cloud2>\n")

# Stereo Image Proc executable
add_executable(stereo_image_proc
  src/stereo_image_proc.cpp
)

target_link_libraries(stereo_image_proc
  disparity
  point_cloud2
  ament_index_cpp::ament_index_cpp)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(stereo_image_proc "stdc++fs")
endif()

ament_target_dependencies(stereo_image_proc
  ${dependencies}
)

install(TARGETS
  stereo_image_proc
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS disparity
  EXPORT export_disparity
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS point_cloud2
  EXPORT export_point_cloud2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_interfaces(
  export_disparity HAS_LIBRARY_TARGET
  export_point_cloud2 HAS_LIBRARY_TARGET
)

ament_export_libraries(
  disparity
  point_cloud2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()