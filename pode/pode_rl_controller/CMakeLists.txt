cmake_minimum_required(VERSION 3.8)
project(pode_rl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(pode_rl_core SHARED
  src/policy_spec.cpp
  src/observation.cpp
  src/onnx_policy.cpp)
target_include_directories(pode_rl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pode_rl_core PUBLIC onnxruntime::onnxruntime yaml-cpp::yaml-cpp)

add_library(${PROJECT_NAME} SHARED src/follow_path_rl.cpp)
target_link_libraries(${PROJECT_NAME} pode_rl_core)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp geometry_msgs nav2_core nav2_costmap_2d nav2_util nav_msgs pluginlib rclcpp
  rclcpp_lifecycle sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

pluginlib_export_plugin_description_file(nav2_core follow_path_rl.xml)

install(TARGETS pode_rl_core ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY policies DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 후보마다 parity 시험. 저장소에는 4b의 policy.onnx만 있어 4b는 꼭 있어야 하고, 나머지는 없으면 추론 비교만 건너뛴다
  foreach(policy store_arena_4b store_arena_c store_arena_acd store_arena_acd2)
    if(policy STREQUAL "store_arena_4b")
      set(onnx_required 1)
    else()
      set(onnx_required 0)
    endif()
    ament_add_gtest(test_parity_${policy} test/test_parity.cpp)
    target_link_libraries(test_parity_${policy} pode_rl_core)
    target_compile_definitions(test_parity_${policy} PRIVATE
      ONNX_REQUIRED=${onnx_required}
      FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/${policy}"
      POLICY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/policies/${policy}"
      NAV_SPEED_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../pode_nav_config/params/nav_speed.yaml")
  endforeach()
  ament_add_gtest(test_observation test/test_observation.cpp)
  target_link_libraries(test_observation pode_rl_core)
  target_compile_definitions(test_observation PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/store_arena_4b"
    POLICY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/policies/store_arena_4b"
    PATH_END_POLICY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/goal_reach_base_4096"
    NAV_SPEED_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../pode_nav_config/params/nav_speed.yaml")
  ament_add_gtest(test_format3 test/test_format3.cpp)
  target_link_libraries(test_format3 pode_rl_core)
  target_compile_definitions(test_format3 PRIVATE
    POLICY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/policies/store_arena_4b"
    FORMAT3_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/format3/format3_cases.json"
    NAV_SPEED_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../pode_nav_config/params/nav_speed.yaml")
endif()

ament_export_include_directories(include)
ament_export_libraries(pode_rl_core ${PROJECT_NAME})
ament_export_dependencies(onnxruntime_vendor yaml_cpp_vendor)
ament_package()
