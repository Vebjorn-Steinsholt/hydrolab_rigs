cmake_minimum_required(VERSION 3.8)
project(rig_web)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/web_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_server
)
install(DIRECTORY html launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
