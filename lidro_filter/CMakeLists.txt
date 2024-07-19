cmake_minimum_required(VERSION 3.5)
project(lidro_filter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lidro_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# compile the filter and input library
# add_subdirectory(src/lib)

# build the filter node
add_library(lidro_filter SHARED src/filter.cpp)
ament_target_dependencies(lidro_filter
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  tf2_ros
  lidro_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
)
target_link_libraries(
  lidro_filter
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)


add_executable(lidro_filter_node src/lidro_node.cpp)
ament_target_dependencies(lidro_filter_node
  rclcpp
)
target_link_libraries(lidro_filter_node lidro_filter)

# install runtime and library files
install(TARGETS lidro_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS lidro_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(lidro_filter
  "lidro_filter::LidroFilter")

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
