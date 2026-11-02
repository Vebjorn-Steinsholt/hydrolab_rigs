cmake_minimum_required(VERSION 3.15)
project(linmot_serial_driver LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(linmot_serial
    src/protocol.cpp
    src/serial_port.cpp
    src/serial_bridge.cpp
    src/linmot_drive.cpp
)
target_include_directories(linmot_serial PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(linmot_serial PUBLIC Threads::Threads)
target_compile_options(linmot_serial PRIVATE -Wall -Wextra)

# Hardware tool: checks the link without moving anything (NMT operational and status only).
add_executable(linmot_probe examples/linmot_probe.cpp)
target_link_libraries(linmot_probe PRIVATE linmot_serial)

# Both tests are self-contained; test_drive simulates the Arduino and the drive on a
# pseudo-terminal (tests/bridge_sim.h).
enable_testing()

add_executable(test_protocol tests/test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE linmot_serial)
add_test(NAME protocol COMMAND test_protocol)

add_executable(test_drive tests/test_drive.cpp)
target_link_libraries(test_drive PRIVATE linmot_serial)
add_test(NAME drive COMMAND test_drive)

install(TARGETS linmot_probe DESTINATION lib/${PROJECT_NAME})
install(TARGETS linmot_serial
    EXPORT linmot_serial_driverTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
# The simulated Arduino and drive, for tests of code that uses this driver (rig_controller).
install(FILES tests/bridge_sim.h DESTINATION include/linmot/testing)

ament_export_targets(linmot_serial_driverTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Threads)
ament_package()
