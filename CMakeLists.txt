cmake_minimum_required(VERSION 3.16)
project(semantic_toponav_nav2_bt CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(semantic_toponav_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  semantic_toponav_msgs
)

add_library(follow_semantic_waypoints_action SHARED
  src/follow_semantic_waypoints_action.cpp
)
target_include_directories(follow_semantic_waypoints_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(follow_semantic_waypoints_action ${dependencies})

# Mark this library as a BT.CPP plugin so nav2_behavior_tree picks it up.
target_compile_definitions(follow_semantic_waypoints_action
  PRIVATE BT_PLUGIN_EXPORT
)

install(TARGETS follow_semantic_waypoints_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # ament_copyright is excluded because it requires a CONTRIBUTING.md
  # at the package root and rejects the standard Apache-2.0 LICENSE
  # text (which has no project-line copyright header). cpplint's
  # legal/copyright check still verifies that every C++ source file
  # carries a Copyright line.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  target_link_libraries(test_plugin_loading follow_semantic_waypoints_action)
  ament_target_dependencies(test_plugin_loading ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(follow_semantic_waypoints_action)
ament_export_dependencies(${dependencies})

ament_package()
