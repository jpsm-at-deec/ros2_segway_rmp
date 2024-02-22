cmake_minimum_required(VERSION 3.8)
project(ros2_segway_rmp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(libsegwayrmp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})



set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${system_INCLUDE_DIRS}
  ${REQUIRED_INCLUDE_DIRS} ${serial_INCLUDE_DIRS} ${libsegwayrmp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} ${thread_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${nav_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} ${system_LIBRARIES}
  ${REQUIRED_LIBRARIES} ${serial_LIBRARIES} ${libsegwayrmp_LIBRARIES}
  ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES} ${thread_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES})

set(LIBS ${ament_cmake_LIBRARIES} ${nav_msgs_LIBRARIES} ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES} ${system_LIBRARIES} ${REQUIRED_LIBRARIES}
  ${serial_LIBRARIES} ${libsegwayrmp_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES}
  ${thread_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES})
rosidl_generate_interfaces(${PROJECT_NAME} #"msg/msg" "msg/FILES"
  "msg/SegwayStatus.msg"
  "msg/SegwayStatusStamped.msg"
  DEPENDENCIES nav_msgs std_msgs geometry_msgs #builtin_interfaces
)

add_executable(ros2_segway_rmp_node src/ros2_segway_rmp_node.cpp)
target_include_directories(ros2_segway_rmp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ros2_segway_rmp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ros2_segway_rmp_node
  "geometry_msgs"
  "libsegwayrmp"
  "nav_msgs"
  "rclcpp"
  "serial"
  "std_msgs"
  "tf2_ros"
  "tf2"
  "builtin_interfaces"
  "Boost"
)

install(TARGETS ros2_segway_rmp_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
