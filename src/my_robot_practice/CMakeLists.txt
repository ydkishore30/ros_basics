cmake_minimum_required(VERSION 3.8)
project(my_robot_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
add_executable(hello_node src/hello_node.cpp)

ament_target_dependencies(hello_node rclcpp std_msgs)
install(TARGETS hello_node DESTINATION lib/${PROJECT_NAME})


add_executable(battery_node src/battery.cpp)
ament_target_dependencies(battery_node rclcpp my_robot_interfaces std_srvs)
install(TARGETS battery_node DESTINATION lib/${PROJECT_NAME})

add_executable(battery_monitor src/battery_monitor.cpp)
ament_target_dependencies(battery_monitor rclcpp my_robot_interfaces)
install(TARGETS battery_monitor DESTINATION lib/${PROJECT_NAME})

add_executable(battery_client src/battery_client.cpp)
ament_target_dependencies(battery_client rclcpp my_robot_interfaces)
install(TARGETS battery_client DESTINATION lib/${PROJECT_NAME})

add_executable(count_listener src/count_listener.cpp)
ament_target_dependencies(count_listener rclcpp std_msgs)
install(TARGETS count_listener DESTINATION lib/${PROJECT_NAME})

add_executable(charger_server src/charger_server.cpp)
ament_target_dependencies(charger_server rclcpp my_robot_interfaces rclcpp_action)
install(TARGETS charger_server DESTINATION lib/${PROJECT_NAME})

add_executable(charger_client src/charger_client.cpp)
ament_target_dependencies(charger_client rclcpp my_robot_interfaces rclcpp_action)
install(TARGETS charger_client DESTINATION lib/${PROJECT_NAME})

add_executable(temperature_sensor src/temperature_sensor.cpp)
ament_target_dependencies(temperature_sensor rclcpp my_robot_interfaces)
install(TARGETS temperature_sensor DESTINATION lib/${PROJECT_NAME})

add_executable(temperature_monitor src/temperature_monitor.cpp)
ament_target_dependencies(temperature_monitor rclcpp my_robot_interfaces)
install(TARGETS temperature_monitor DESTINATION lib/${PROJECT_NAME})

add_executable(config_publisher src/config_publisher.cpp)
ament_target_dependencies(config_publisher rclcpp std_msgs)
install(TARGETS config_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(big_publisher src/big_publisher.cpp)
ament_target_dependencies(big_publisher rclcpp std_msgs)
install(TARGETS big_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(zero_copy_demo src/zero_copy_demo.cpp)
ament_target_dependencies(zero_copy_demo rclcpp std_msgs)
install(TARGETS zero_copy_demo DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

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
