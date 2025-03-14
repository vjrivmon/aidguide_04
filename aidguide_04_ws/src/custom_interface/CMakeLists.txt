cmake_minimum_required(VERSION 3.8)
project(custom_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Definir las interfaces personalizadas
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/NavigationCommand.msg"
  "srv/GetRobotStatus.srv"
  "srv/SetNavigationGoal.srv"
)

ament_package() 