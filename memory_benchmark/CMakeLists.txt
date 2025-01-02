cmake_minimum_required(VERSION 3.5)
project(memory_benchmark)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate all ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
 "msg/M0MB.msg"
 "msg/M1.msg"
 "msg/M2.msg"
 "msg/M3.msg"
 "msg/M4.msg"
 "msg/M5.msg"
 "msg/M6.msg"
 "msg/M7.msg"
 "msg/M8.msg"
 "msg/M9.msg"
 "msg/M10.msg"
 "msg/M11.msg"
 "msg/M12.msg"
 "msg/M13.msg"
 "msg/M14.msg"
 "msg/M15.msg"
 "msg/M16.msg"
 "msg/M17.msg"
 "msg/M18.msg"
 "msg/M19.msg"
 "msg/M20.msg"
 "msg/M21.msg"
 "msg/M22.msg"
 "msg/M23.msg"
 "msg/M24.msg"
 "msg/M25.msg"
 "msg/M26.msg"
 "msg/M27.msg"
 "msg/M28.msg"
 "msg/M29.msg"
 "msg/M30.msg"
 "msg/M31.msg"
 "msg/M32.msg"
 "msg/M33.msg"
 "msg/M34.msg"
 "msg/M35.msg"
 "msg/M36.msg"
 "msg/M37.msg"
 "msg/M38.msg"
 "msg/M39.msg"
 "msg/M40.msg"
 "msg/M41.msg"
 "msg/M42.msg"
 "msg/M43.msg"
 "msg/M44.msg"
 "msg/M45.msg"
 "msg/M46.msg"
 "msg/M47.msg"
 "msg/M48.msg"
 "msg/M49.msg"
 "msg/M50.msg"
)

# add_executable(memory-test src/memory-test.cpp)
# target_include_directories(memory-test PRIVATE include)
# ament_target_dependencies(memory-test rclcpp memory_msgs std_srvs)

add_executable(default_nodes src/default_nodes.cpp)
target_include_directories(default_nodes PRIVATE include)
ament_target_dependencies(default_nodes rclcpp)

add_executable(nodes_params_off src/nodes_params_off.cpp)
target_include_directories(nodes_params_off PRIVATE include)
ament_target_dependencies(nodes_params_off rclcpp)

add_executable(default_subs_params_off src/default_subs_params_off.cpp)
target_include_directories(default_subs_params_off PRIVATE include)
ament_target_dependencies(default_subs_params_off rclcpp)
rosidl_target_interfaces(default_subs_params_off ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(default_pubs_params_off src/default_pubs_params_off.cpp)
target_include_directories(default_pubs_params_off PRIVATE include)
ament_target_dependencies(default_pubs_params_off rclcpp)
rosidl_target_interfaces(default_pubs_params_off ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(default_clients_params_off src/default_clients_params_off.cpp)
target_include_directories(default_clients_params_off PRIVATE include)
ament_target_dependencies(default_clients_params_off rclcpp std_srvs)

add_executable(default_services_params_off src/default_services_params_off.cpp)
target_include_directories(default_services_params_off PRIVATE include)
ament_target_dependencies(default_services_params_off rclcpp std_srvs)

add_executable(nodes_params_off_logging_on src/nodes_params_off_logging_on.cpp)
target_include_directories(nodes_params_off_logging_on PRIVATE include)
ament_target_dependencies(nodes_params_off_logging_on rclcpp)

add_executable(pub_sub_diff_topic src/pub_sub_diff_topic.cpp)
target_include_directories(pub_sub_diff_topic PRIVATE include)
ament_target_dependencies(pub_sub_diff_topic rclcpp)
rosidl_target_interfaces(pub_sub_diff_topic ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_sub_same_topic src/pub_sub_same_topic.cpp)
target_include_directories(pub_sub_same_topic PRIVATE include)
ament_target_dependencies(pub_sub_same_topic rclcpp)
rosidl_target_interfaces(pub_sub_same_topic ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_sub_diff_msg_type src/pub_sub_diff_msg_type.cpp)
target_include_directories(pub_sub_diff_msg_type PRIVATE include)
ament_target_dependencies(pub_sub_diff_msg_type rclcpp)
rosidl_target_interfaces(pub_sub_diff_msg_type ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(cli_serv_diff_topics src/cli_serv_diff_topics.cpp)
target_include_directories(cli_serv_diff_topics PRIVATE include)
ament_target_dependencies(cli_serv_diff_topics rclcpp std_srvs)

add_executable(cli_serv_same_topics src/cli_serv_same_topics.cpp)
target_include_directories(cli_serv_same_topics PRIVATE include)
ament_target_dependencies(cli_serv_same_topics rclcpp std_srvs)

add_executable(pub_sub_big_history_size src/pub_sub_big_history_size.cpp)
target_include_directories(pub_sub_big_history_size PRIVATE include)
ament_target_dependencies(pub_sub_big_history_size rclcpp)
rosidl_target_interfaces(pub_sub_big_history_size ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pub_sub_big_message_size src/pub_sub_big_message_size.cpp)
target_include_directories(pub_sub_big_message_size PRIVATE include)
ament_target_dependencies(pub_sub_big_message_size rclcpp)
rosidl_target_interfaces(pub_sub_big_message_size ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  default_nodes
  nodes_params_off
  default_subs_params_off
  default_pubs_params_off
  default_clients_params_off
  default_services_params_off
  nodes_params_off_logging_on
  pub_sub_diff_topic
  pub_sub_same_topic
  pub_sub_diff_msg_type
  cli_serv_diff_topics
  cli_serv_same_topics
  pub_sub_big_history_size
  pub_sub_big_message_size
  DESTINATION lib/${PROJECT_NAME}
)

# Install the entire comms_benchmark directory
install(DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
