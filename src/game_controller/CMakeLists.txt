cmake_minimum_required(VERSION 3.8)
project(game_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GameControlData.msg"
  "msg/RobotInfo.msg"
  "msg/TeamInfo.msg"
)

add_executable(game_controller_node src/main.cpp src/game_controller_node.cpp)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(game_controller_node ${typesupport_target})

ament_target_dependencies(game_controller_node rclcpp)

target_include_directories(game_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(TARGETS
  game_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
