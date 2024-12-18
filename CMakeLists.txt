cmake_minimum_required(VERSION 3.5)
project(starling_offboard_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(message_filters REQUIRED)

set(target_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  px4_msgs 
  GeographicLib
  nav_msgs
  rosbag2_cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(starling_offboard_cpp src/starling_offboard.cpp)
ament_target_dependencies(starling_offboard_cpp  ${target_dependencies})
target_link_libraries(starling_offboard_cpp Eigen3::Eigen)
install(TARGETS starling_offboard_cpp DESTINATION lib/${PROJECT_NAME})

add_executable(starling_transform_test_cpp src/starling_transform_test.cpp)
ament_target_dependencies(starling_transform_test_cpp ${target_dependencies})
target_link_libraries(starling_transform_test_cpp Eigen3::Eigen)
install(TARGETS starling_transform_test_cpp DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
