cmake_minimum_required(VERSION 3.5)
project(car_sim_gazebo)

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
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(car_plugin SHARED
  src/car_plugin.cpp
)
ament_target_dependencies(car_plugin
  rclcpp
  gazebo_ros
  tf2_ros
  nav_msgs
  geometry_msgs
  example_interfaces
)
target_include_directories(car_plugin PUBLIC
  include
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(car_plugin
  ${GAZEBO_LIBRARIES}
)

### Install Rules
install(
  TARGETS
    car_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
