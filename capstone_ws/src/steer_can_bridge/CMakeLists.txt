cmake_minimum_required(VERSION 3.8)
project(steer_can_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(capstone_msgs REQUIRED)

# ROS 무관 코어 (프로토콜 + 상태기계 + 소켓) — 노드와 테스트가 공유
add_library(steer_can_core STATIC src/supervisor.cpp src/socketcan.cpp)
target_include_directories(steer_can_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(steer_can_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(steer_can_bridge_node src/bridge_node.cpp)
target_link_libraries(steer_can_bridge_node steer_can_core)
ament_target_dependencies(steer_can_bridge_node rclcpp std_srvs capstone_msgs)

install(TARGETS steer_can_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Python(Steer_motor/steer_can) 과 같은 테스트 벡터 파일
  get_filename_component(VECTORS_CSV
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Steer_motor/steer_can/tests/rs02_vectors.csv" ABSOLUTE)
  ament_add_gtest(test_rs02 test/test_rs02.cpp)
  target_link_libraries(test_rs02 steer_can_core)
  target_compile_definitions(test_rs02 PRIVATE VECTORS_CSV="${VECTORS_CSV}")
  ament_add_gtest(test_supervisor test/test_supervisor.cpp)
  target_link_libraries(test_supervisor steer_can_core)
endif()

ament_package()
