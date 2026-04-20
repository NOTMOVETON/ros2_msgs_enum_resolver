cmake_minimum_required(VERSION 3.8)
project(ros2_msgs_enum_resolver VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp    REQUIRED)

# ---------------------------------------------------------------------------
# C++ shared library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/ros2_msgs_enum_resolver.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
)

# ---------------------------------------------------------------------------
# Install C++
# ---------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------------------------
# Install Python package
# ---------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ros2_msgs_enum_resolver
)

# ---------------------------------------------------------------------------
# Install scripts (enum_echo entry point)
# ---------------------------------------------------------------------------
# lib/${PROJECT_NAME}/ — for `ros2 run ros2_msgs_enum_resolver enum_echo`
# bin/                 — for direct `enum_echo` invocation (always on PATH)
install(
  PROGRAMS scripts/enum_echo
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/enum_echo
  DESTINATION bin
)

# Tab completion:
#   enum_echo_completion.bash — ament env hook (sourced via setup.bash)
#   enum_echo.bash-completion — static bash completion file for XDG_DATA_DIRS lookup
install(
  FILES env-hooks/enum_echo_completion.bash
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES env-hooks/enum_echo.bash-completion
  RENAME enum_echo
  DESTINATION share/bash-completion/completions
)
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/enum_echo_completion.dsv.in"
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp)
ament_export_include_directories(include)

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------
find_package(rclcpp     REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++ — basic usage (не нода, просто печатает и выходит)
add_executable(example_basic_usage examples/cpp/basic_usage.cpp)
target_link_libraries(example_basic_usage ${PROJECT_NAME})

# C++ — publisher и subscriber ноды
add_executable(example_publisher  examples/cpp/publisher_node.cpp)
add_executable(example_subscriber examples/cpp/subscriber_node.cpp)
foreach(tgt example_publisher example_subscriber)
  target_link_libraries(${tgt} ${PROJECT_NAME})
  ament_target_dependencies(${tgt} rclcpp sensor_msgs)
endforeach()

install(TARGETS
  example_basic_usage
  example_publisher
  example_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Python — устанавливаем как исполняемые скрипты
install(PROGRAMS
  examples/python/basic_usage.py
  examples/python/publisher_node.py
  examples/python/subscriber_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto    REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest  REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}_cpp
    test/test_ros2_msgs_enum_resolver.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_cpp ${PROJECT_NAME})
  ament_target_dependencies(test_${PROJECT_NAME}_cpp ament_index_cpp)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_${PROJECT_NAME}_py
    test/test_ros2_msgs_enum_resolver.py
  )
endif()

ament_package()