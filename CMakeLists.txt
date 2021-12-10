cmake_minimum_required(VERSION 3.0.2)
project(task_manager)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  detection_msgs
  pos_collector_msgs
  std_msgs
  std_srvs
  visualization_msgs
  fast_planner_server
  transform_services
  motion_controller
  autotakeoff
  nodes_monitor_msgs
  plan_env
  sensor_msgs
  qr_detector_msgs
)

add_message_files(
  FILES
  DroneStatus.msg
)

add_service_files(
  FILES
  Start.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  detection_msgs
  visualization_msgs
  pos_collector_msgs
)

catkin_package(
)

add_custom_target(task_manager
   ALL
   COMMAND cd ${CMAKE_BINARY_DIR}/../.. && cargo build -p task_manager --release
   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/../../target/release/task_manager ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/task_manager
   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/../../target/release/test_flew_into_hole_detection ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_flew_into_hole_detection
   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/../../target/release/test_fly_into_hole ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_fly_into_hole
   COMMENT "Building task_manager..."
)
