cmake_minimum_required(VERSION 3.8)
project(ur7e_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecutePrimitive.action"
  "srv/DetectObject.srv"
  "srv/LocateObject.srv"
  DEPENDENCIES geometry_msgs action_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
