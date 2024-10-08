cmake_minimum_required(VERSION 3.8)
project(mavsdk_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(privyaznik_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(euler_angles_lib REQUIRED)
find_package(pid_regulator_lib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(precision_landing_transmitter src/precision_landing_transmitter.cpp)
target_link_libraries(precision_landing_transmitter MAVSDK::mavsdk zmq pid_regulator_lib::pid_regulator_lib)
ament_target_dependencies(precision_landing_transmitter  rclcpp geometry_msgs std_msgs sensor_msgs tf2_geometry_msgs mavros_msgs privyaznik_msgs tf2 rclcpp_action)

add_executable(json_cmd_client src/json_cmd_client.cpp)
target_link_libraries(json_cmd_client MAVSDK::mavsdk zmq)
ament_target_dependencies(json_cmd_client rclcpp geometry_msgs std_msgs sensor_msgs tf2_geometry_msgs mavros_msgs privyaznik_msgs tf2 rclcpp_action)

add_executable(try_move src/try_move.cpp)
target_link_libraries(try_move MAVSDK::mavsdk)
ament_target_dependencies(try_move rclcpp geometry_msgs rclcpp privyaznik_msgs)


add_executable(try_takeoff src/try_takeoff.cpp)
target_link_libraries(try_takeoff MAVSDK::mavsdk)
ament_target_dependencies(try_takeoff rclcpp geometry_msgs rclcpp privyaznik_msgs)


add_executable(try_land src/try_land.cpp)
target_link_libraries(try_land MAVSDK::mavsdk)
ament_target_dependencies(try_land rclcpp geometry_msgs rclcpp privyaznik_msgs)


add_executable(client_commands_takeoff src/client_commands_takeoff.cpp)
target_link_libraries(client_commands_takeoff MAVSDK::mavsdk)
ament_target_dependencies(client_commands_takeoff rclcpp geometry_msgs rclcpp privyaznik_msgs)


add_executable(client_commands_land src/client_commands_land.cpp)
target_link_libraries(client_commands_land MAVSDK::mavsdk)
ament_target_dependencies(client_commands_land rclcpp geometry_msgs rclcpp privyaznik_msgs)


add_executable(action_client_takeoff src/action_client_takeoff.cpp)
target_link_libraries(action_client_takeoff MAVSDK::mavsdk)
ament_target_dependencies(action_client_takeoff rclcpp geometry_msgs rclcpp privyaznik_msgs rclcpp_action)


add_executable(action_client_land src/action_client_land.cpp)
target_link_libraries(action_client_land MAVSDK::mavsdk)
ament_target_dependencies(action_client_land rclcpp geometry_msgs rclcpp privyaznik_msgs rclcpp_action)


add_executable(action_client_move src/action_client_move.cpp)
target_link_libraries(action_client_move MAVSDK::mavsdk)
ament_target_dependencies(action_client_move rclcpp geometry_msgs rclcpp privyaznik_msgs rclcpp_action)


add_executable(action_service src/action_service.cpp)
target_link_libraries(action_service MAVSDK::mavsdk)
ament_target_dependencies(action_service rclcpp geometry_msgs rclcpp privyaznik_msgs rclcpp_action)


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

install(TARGETS
  precision_landing_transmitter
  json_cmd_client
  try_move
  try_land
  try_takeoff
  client_commands_land
  client_commands_takeoff
  action_client_takeoff
  action_service
  action_client_land
  action_client_move
  DESTINATION lib/${PROJECT_NAME}
)


install(
    TARGETS precision_landing_transmitter
    EXPORT precision_landing_transmitter
    LIBRARY DESTINATION ${COLCON_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${COLCON_GLOBAL_LIB_DESTINATION}
    RUNTIME DESTINATION  ${COLCON_GLOBAL_LIB_DESTINATION}
    INCLUDES DESTINATION include
)

install(TARGETS precision_landing_transmitter
DESTINATION lib/${PROJECT_NAME}/)

ament_export_targets(precision_landing_transmitter HAS_LIBRARY_TARGET)


ament_package()
