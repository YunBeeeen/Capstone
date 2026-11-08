cmake_minimum_required(VERSION 3.8)
project(swerve_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(capstone_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

# ---- ROS 무관 기구학 (IK / 가동범위 접기 / 정렬 게이트 / FK) ----
# 다른 C++ 패키지:  find_package(swerve_controller) + target_link_libraries(x swerve_controller::swerve_core)
#                   #include "swerve_controller/kinematics.hpp"
add_library(swerve_core STATIC src/kinematics.cpp)
target_include_directories(swerve_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
set_target_properties(swerve_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS swerve_core EXPORT export_swerve_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_swerve_core HAS_LIBRARY_TARGET)

# ---- Python:  import swerve_kinematics  (source install/setup.bash 후) ----
pybind11_add_module(swerve_kinematics src/python_bindings.cpp)
target_link_libraries(swerve_kinematics PRIVATE swerve_core)
install(TARGETS swerve_kinematics
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)

# ---- ROS2 노드 ----
add_executable(swerve_node src/swerve_node.cpp)
target_link_libraries(swerve_node swerve_core)
ament_target_dependencies(swerve_node rclcpp std_srvs geometry_msgs capstone_msgs)

install(TARGETS swerve_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics swerve_core)
  # 빌드 폴더의 바인딩 모듈을 그대로 import 해서 시험
  add_test(NAME test_bindings
    COMMAND ${Python3_EXECUTABLE} -m unittest -v test_bindings
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
  set_tests_properties(test_bindings PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}")
endif()

ament_package()
