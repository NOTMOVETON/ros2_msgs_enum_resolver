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
install(
  PROGRAMS scripts/enum_echo
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp)
ament_export_include_directories(include)

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