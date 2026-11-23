cmake_minimum_required(VERSION 3.8)
project(wave_probe)

find_package(ament_cmake REQUIRED)

# Python only: the node and the module it imports, installed without .py as rig_controller does.
install(PROGRAMS src/wave_probe_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wave_probe_node)
install(FILES src/probe_serial.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_probe_serial test/test_probe_serial.py)
  ament_add_pytest_test(test_wave_probe_node test/test_wave_probe_node.py)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)   # until the licence is set (TODO 32)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
