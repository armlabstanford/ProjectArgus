cmake_minimum_required(VERSION 3.8)
project(argus_mahoney)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mahoney_node src/mahoney_node.cpp)
target_include_directories(mahoney_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(mahoney_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_srvs
)

install(TARGETS mahoney_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
