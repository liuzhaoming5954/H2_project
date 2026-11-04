cmake_minimum_required(VERSION 3.22)
project(h2_navigation_foundation LANGUAGES CXX)

include(CTest)
add_subdirectory(src/h2_core)

if(BUILD_TESTING)
  add_subdirectory(tests/core)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME tooling COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_SOURCE_DIR}/tests/tools -p "test_*.py")
  set_tests_properties(tooling PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  add_test(NAME readonly_bringup_config COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_SOURCE_DIR}/tests/bringup -p "test_*.py")
  set_tests_properties(readonly_bringup_config PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  add_test(NAME supervisor_health COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_SOURCE_DIR}/src/h2_supervisor/test -p "test_*.py")
  set_tests_properties(supervisor_health PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/src/h2_supervisor")
endif()
