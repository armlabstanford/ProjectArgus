cmake_minimum_required(VERSION 3.8)
project(argus_server)

# -------------------------
# Compiler settings
# -------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# -------------------------
# Find ROS 2 dependencies
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# -------------------------
# OpenSim dependency
# -------------------------
find_package(OpenSim REQUIRED)

# If OpenSim is not in a standard location, you may need:
#list(APPEND CMAKE_PREFIX_PATH "~/to/opensim/install")

# -------------------------
# Executable
# -------------------------
add_executable(argus_server
  src/argus_server.cpp
)

add_executable(fake_data_gen
  src/fake_data_gen.cpp
)

# -------------------------
# Include directories
# -------------------------

target_link_libraries(argus_server
  ${OpenSim_LIBRARIES}
)

target_include_directories(argus_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenSim_INCLUDE_DIRS}
  #${OpenSim_ROOT_DIR}/sdk/spdlog/include
)

target_include_directories(fake_data_gen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenSim_INCLUDE_DIRS}
)

# -------------------------
# Link libraries
# -------------------------

ament_target_dependencies(argus_server
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(fake_data_gen
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
)

# -------------------------
# Install targets
# -------------------------
install(TARGETS argus_server
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS fake_data_gen
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------
# Ament package export
# -------------------------
ament_package()