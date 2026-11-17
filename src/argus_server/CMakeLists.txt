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
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

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

add_executable(pose_monitor
  src/pose_monitor.cpp
)
ament_target_dependencies(pose_monitor
  rclcpp
  nav_msgs
)

add_executable(argus_debug
  src/argus_debug.cpp
)
ament_target_dependencies(argus_debug
  rclcpp
  std_msgs
)

add_executable(pose_bones
  src/pose_bones.cpp
)
target_link_libraries(pose_bones
  ${OpenSim_LIBRARIES}
)
target_include_directories(pose_bones BEFORE PRIVATE
  ${OpenSim_INCLUDE_DIRS}
  "${OpenSim_ROOT_DIR}/sdk/spdlog/include"
)
target_compile_options(pose_bones PRIVATE
  "SHELL:-USPDLOG_FMT_EXTERNAL"
  "SHELL:-USPDLOG_SHARED_LIB"
  "SHELL:-UFMT_SHARED"
  "SHELL:-UFMT_LOCALE"
)
add_custom_command(TARGET pose_bones PRE_LINK
  COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rename_spdlog_symbols.sh"
          "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/pose_bones.dir/src/pose_bones.cpp.o"
  VERBATIM
)

# -------------------------
# Include directories
# -------------------------

target_link_libraries(argus_server
  ${OpenSim_LIBRARIES}
)

# OpenSim's spdlog 1.15 symbols are renamed with an osrt_ prefix inside the
# installed libraries so they do not interpose on ROS Humble's libspdlog 1.9.
# Apply the same rename to this translation unit before linking.
add_custom_command(TARGET argus_server PRE_LINK
  COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rename_spdlog_symbols.sh"
          "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/argus_server.dir/src/argus_server.cpp.o"
  VERBATIM
)

target_include_directories(argus_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenSim_INCLUDE_DIRS}
)

# OpenSim 4.6 is built against the spdlog 1.15 copy under
# ${OpenSim_ROOT_DIR}/sdk/spdlog (bundled fmt). ROS Humble finds the system
# spdlog 1.9 package, which uses external fmt 8 and does not ship
# spdlog/fmt/bundled/base.h. Search OpenSim's headers first, and drop the
# system preprocessor flags so Logger.h matches the library OpenSim linked.
target_include_directories(argus_server BEFORE PRIVATE
  "${OpenSim_ROOT_DIR}/sdk/spdlog/include"
)
target_compile_options(argus_server PRIVATE
  "SHELL:-USPDLOG_FMT_EXTERNAL"
  "SHELL:-USPDLOG_SHARED_LIB"
  "SHELL:-UFMT_SHARED"
  "SHELL:-UFMT_LOCALE"
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
  std_msgs
  tf2_msgs
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


install(TARGETS fake_data_gen pose_monitor argus_debug pose_bones
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include/
  )
endif()

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