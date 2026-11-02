cmake_minimum_required(VERSION 3.15)
project(mgate5101_driver LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(mgate5101
    src/modbus_tcp.cpp
    src/process_image.cpp
    src/mgate_driver.cpp
    src/indradrive.cpp
)
target_include_directories(mgate5101 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(WIN32)
    target_link_libraries(mgate5101 PUBLIC ws2_32)
    target_compile_definitions(mgate5101 PUBLIC _WIN32_WINNT=0x0601 NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

if(MSVC)
    target_compile_options(mgate5101 PRIVATE /W4 /permissive-)
else()
    target_compile_options(mgate5101 PRIVATE -Wall -Wextra)
endif()

add_executable(mgate_example examples/example_main.cpp)
target_link_libraries(mgate_example PRIVATE mgate5101)

# Hardware tool: measures cycle times to choose poll_interval_ms. Writes zeros (drive off).
add_executable(poll_rate_probe examples/poll_rate_probe.cpp)
target_link_libraries(poll_rate_probe PRIVATE mgate5101)

# test_mapping is self-contained.
# test_driver removed from build: requires the Python stub server
# (tests/stub_server.py, listens on 127.0.0.1:1502) and is not needed
# to consume this driver from rig_controller. Source kept in
# tests/ for reference; run manually with the stub if ever needed.
enable_testing()

add_executable(test_mapping tests/test_mapping.cpp)
target_link_libraries(test_mapping PRIVATE mgate5101)
add_test(NAME mapping COMMAND test_mapping)

install(TARGETS poll_rate_probe DESTINATION lib/${PROJECT_NAME})
install(TARGETS mgate5101
    EXPORT mgate5101_driverTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY resources/profibus_gsd
    DESTINATION share/${PROJECT_NAME}/resources/profibus_gsd
)

ament_export_targets(mgate5101_driverTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()