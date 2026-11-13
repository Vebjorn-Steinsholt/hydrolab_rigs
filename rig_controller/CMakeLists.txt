cmake_minimum_required(VERSION 3.8)
project(rig_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rig_interfaces REQUIRED)
find_package(bondcpp REQUIRED)
find_package(mgate5101_driver REQUIRED)
find_package(linmot_serial_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(controller_node
  src/controller_node.cpp
  src/indradrive_actuator.cpp
  src/linmot_actuator.cpp
  src/wave_math.cpp
  src/wave_trajectory.cpp
)
ament_target_dependencies(controller_node
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rig_interfaces
  bondcpp
  std_msgs
  mgate5101_driver
)

add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  bondcpp
  rig_interfaces
  std_msgs
  std_srvs
)

install(TARGETS controller_node bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
# Test tool: stands in for the controller without hardware (see the script's docstring).
install(PROGRAMS test/fake_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_controller
)
# Plays pregenerated motion files (.dat) as move_<rig> goals (see the script's docstring).
install(PROGRAMS src/play_dat.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME play_dat
)
# Generates motion files (.dat + run table) for play_dat (see the script's docstring).
install(PROGRAMS src/generate_waves.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_waves
)
# The same for an oscillator: stroke instead of wave height (see the script's docstring).
install(PROGRAMS src/generate_oscillations.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_oscillations
)
install(PROGRAMS src/soft_stop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME soft_stop
)
# Transfer function and gain from a run and a catman record (see the script's docstring).
install(PROGRAMS src/transfer_function.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME transfer_function
)
# The generators record the repository's commit (source_commit); an installed copy finds the
# repository through this file.
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/source_dir.txt "${CMAKE_CURRENT_SOURCE_DIR}\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/source_dir.txt
  DESTINATION lib/${PROJECT_NAME}
)

# Imported by play_dat and the generators; the programs are installed without .py.
install(FILES src/play_dat.py src/rig_limits.py src/motion_files.py src/generate_waves.py
  DESTINATION lib/${PROJECT_NAME}
)


target_link_libraries(controller_node
  mgate5101_driver::mgate5101
  linmot_serial_driver::linmot_serial
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_indradrive_actuator
    test/test_indradrive_actuator.cpp
    src/indradrive_actuator.cpp
  )
  target_include_directories(test_indradrive_actuator PRIVATE src)
  ament_target_dependencies(test_indradrive_actuator
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(test_indradrive_actuator mgate5101_driver::mgate5101)

  # Runs the actuator against the simulated Arduino and drive from linmot_serial_driver.
  ament_add_gtest(test_linmot_actuator
    test/test_linmot_actuator.cpp
    src/linmot_actuator.cpp
  )
  target_include_directories(test_linmot_actuator PRIVATE src)
  ament_target_dependencies(test_linmot_actuator
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(test_linmot_actuator linmot_serial_driver::linmot_serial)

  ament_add_gtest(test_wave_trajectory
    test/test_wave_trajectory.cpp
    src/wave_math.cpp
    src/wave_trajectory.cpp
  )
  target_include_directories(test_wave_trajectory PRIVATE src)

  ament_add_gtest(test_controller_node
    test/test_controller_node.cpp
    src/indradrive_actuator.cpp
    src/linmot_actuator.cpp
    src/wave_math.cpp
    src/wave_trajectory.cpp
  )
  target_include_directories(test_controller_node PRIVATE src)
  ament_target_dependencies(test_controller_node
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    rig_interfaces
    bondcpp
    std_msgs
  )
  target_link_libraries(test_controller_node
    mgate5101_driver::mgate5101
    linmot_serial_driver::linmot_serial
  )

  # Runs the bridge against a fake move_wavemaker server (TODO 16).
  ament_add_gtest(test_bridge_node test/test_bridge_node.cpp TIMEOUT 120)
  ament_target_dependencies(test_bridge_node
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    bondcpp
    rig_interfaces
    std_msgs
    std_srvs
  )
endif()

ament_package()