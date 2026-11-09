cmake_minimum_required(VERSION 3.16)
project(planner_adapter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# The core is a static library linked into the shared Nav2 plugin.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# grid_graph is a plain CMake library kept next to this package. In Docker it is copied outside
# the colcon src/ folder (so colcon doesn't build it twice) and this path is overridden.
set(GRID_GRAPH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../grid_graph" CACHE PATH "grid_graph source directory")
add_subdirectory(${GRID_GRAPH_DIR} ${CMAKE_CURRENT_BINARY_DIR}/grid_graph)

# The lab's planning library (git submodule at the repo root) for Rule/Rulebook/RulebookCost.
# EXCLUDE_FROM_ALL builds only core_lib (what we link), not its examples or test_suite.
# core_lib inherits CMAKE_POSITION_INDEPENDENT_CODE from above, so it can go into the shared plugin.
set(PLANNING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../planning" CACHE PATH "planning library source directory")
add_subdirectory(${PLANNING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/planning EXCLUDE_FROM_ALL)
if(WIN32)
  # planning builds as strict C++20, where MinGW's <cmath> hides M_PI unless asked for it.
  target_compile_definitions(core_lib PRIVATE _USE_MATH_DEFINES)
endif()

# yaml-cpp for rulebook files: the system one if there is one (the ROS image has 0.7), otherwise a
# pinned release downloaded at configure time (e.g. offline builds on Windows).
find_package(yaml-cpp QUIET)
if(NOT TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
  message(STATUS "yaml-cpp not found; fetching 0.8.0")
  include(FetchContent)
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
  set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  # yaml-cpp 0.8.0 asks for CMake 3.4, which CMake 4 refuses without this.
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  FetchContent_Declare(yaml-cpp
    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
    URL_HASH SHA256=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16)
  FetchContent_MakeAvailable(yaml-cpp)
endif()
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# ---- Core: no ROS. Planner interface, registry, shared pipeline, rulebooks, planners. ----
add_library(planner_adapter_core STATIC
  src/pipeline.cpp
  src/planner_registry.cpp
  src/rule_metrics.cpp
  src/rulebook_config.cpp
  src/planners/dijkstra_planner.cpp
  src/planners/rulebook_planner.cpp
  # Add new planners here.
)
target_include_directories(planner_adapter_core
  PUBLIC include
  PRIVATE src
)
target_link_libraries(planner_adapter_core PUBLIC grid_graph core_lib PRIVATE ${YAML_CPP_TARGET})

# ---- Offline comparison tool: runs planners on a saved map, writes the same CSV. ----
add_executable(compare_planners src/tools/compare_planners.cpp)
target_link_libraries(compare_planners PRIVATE planner_adapter_core)

# ---- Nav2 plugin: only when building inside a ROS 2 workspace. ----
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(nav2_core REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(pluginlib REQUIRED)

  set(dependencies rclcpp rclcpp_lifecycle geometry_msgs nav_msgs nav2_core nav2_costmap_2d tf2_ros pluginlib)

  add_library(planner_adapter_plugin SHARED src/nav2_adapter.cpp)
  target_link_libraries(planner_adapter_plugin planner_adapter_core)
  ament_target_dependencies(planner_adapter_plugin ${dependencies})
  target_compile_definitions(planner_adapter_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  pluginlib_export_plugin_description_file(nav2_core planner_adapter_plugin.xml)

  install(TARGETS planner_adapter_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS compare_planners RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(FILES planner_adapter_plugin.xml DESTINATION share/${PROJECT_NAME})

  ament_export_dependencies(${dependencies})
  ament_package()
endif()
