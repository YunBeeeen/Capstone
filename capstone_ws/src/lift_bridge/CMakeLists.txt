cmake_minimum_required(VERSION 3.8)
project(lift_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/lift_bridge_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/lift_logic.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # 동료 Pico 펌웨어 (capstone_ws/Lift/firmware/main.py) 의 PositionController 와 붙여서 검증
  add_test(NAME test_lift_logic COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lift_logic.py)
endif()

ament_package()
