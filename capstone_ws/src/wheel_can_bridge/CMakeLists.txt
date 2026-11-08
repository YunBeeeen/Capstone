cmake_minimum_required(VERSION 3.8)
project(wheel_can_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(capstone_msgs REQUIRED)

# ROS 무관 코어 (프로토콜 + 상태기계 + 소켓) — 노드와 테스트가 공유
add_library(wheel_can_core STATIC src/supervisor.cpp src/socketcan.cpp)
target_include_directories(wheel_can_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(wheel_can_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(wheel_can_bridge_node src/bridge_node.cpp)
target_link_libraries(wheel_can_bridge_node wheel_can_core)
ament_target_dependencies(wheel_can_bridge_node rclcpp std_srvs capstone_msgs)

install(TARGETS wheel_can_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Python(wheel_can) 과 같은 테스트 벡터 파일
  get_filename_component(VECTORS_CSV
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Wheel_motor/wheel_can/tests/cansimple_vectors.csv" ABSOLUTE)
  ament_add_gtest(test_cansimple test/test_cansimple.cpp)
  target_link_libraries(test_cansimple wheel_can_core)
  target_compile_definitions(test_cansimple PRIVATE VECTORS_CSV="${VECTORS_CSV}")
  ament_add_gtest(test_supervisor test/test_supervisor.cpp)
  target_link_libraries(test_supervisor wheel_can_core)
endif()

ament_package()
