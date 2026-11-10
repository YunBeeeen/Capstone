cmake_minimum_required(VERSION 3.8)
project(capstone_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# 가상 패드 (pygame) — joystick_math.py 는 같은 폴더에서 import
install(PROGRAMS scripts/virtual_pad.py scripts/gamepad_teleop.py scripts/udp_teleop_bridge.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/joystick_math.py scripts/gamepad_map.py scripts/udp_packet.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_joystick_math COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_joystick_math.py)
  set_tests_properties(test_joystick_math PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  add_test(NAME test_gamepad_map COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gamepad_map.py)
  set_tests_properties(test_gamepad_map PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  add_test(NAME test_directions COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_directions.py)
  set_tests_properties(test_directions PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  add_test(NAME test_udp_packet COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_udp_packet.py)
  set_tests_properties(test_udp_packet PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts")
endif()

ament_package()
