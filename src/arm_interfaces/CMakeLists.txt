cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Order matters only in that every .srv referencing Command must be listed in
# the same call — rosidl resolves intra-package references itself.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Command.msg"
  "srv/ParseIntent.srv"
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No source files to copyright-check beyond CMakeLists/package.xml; the
  # repo ships a LICENSE file rather than per-file headers.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
