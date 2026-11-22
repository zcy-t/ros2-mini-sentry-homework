cmake_minimum_required(VERSION 3.10)
project(mini_sentry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mini_sentry_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(chassis_sim src/chassis_sim.cpp)
target_link_libraries(chassis_sim PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  tf2::tf2
  tf2_ros::tf2_ros)

add_executable(referee_sim src/referee_sim.cpp)
target_link_libraries(referee_sim PUBLIC
  rclcpp::rclcpp
  mini_sentry_interfaces::mini_sentry_interfaces)

add_executable(goto_server src/goto_server.cpp)
target_link_libraries(goto_server PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  mini_sentry_interfaces::mini_sentry_interfaces)

add_executable(sentry_brain src/sentry_brain.cpp)
target_link_libraries(sentry_brain PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  mini_sentry_interfaces::mini_sentry_interfaces)

# 作业 2：在线调参
add_executable(cmd_test src/cmd_test.cpp)
target_link_libraries(cmd_test PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs)

add_executable(tf_demo src/tf_demo.cpp)
target_link_libraries(tf_demo PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs)

# 作业 4：收图像、画框、再发出去
add_executable(image_debug src/image_debug.cpp)
target_link_libraries(image_debug PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  cv_bridge::cv_bridge
  ${OpenCV_LIBS})
target_include_directories(image_debug PUBLIC ${OpenCV_INCLUDE_DIRS})

install(TARGETS
  chassis_sim referee_sim goto_server sentry_brain cmd_test tf_demo image_debug
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
