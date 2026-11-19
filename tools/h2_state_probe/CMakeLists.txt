cmake_minimum_required(VERSION 3.22)
project(h2_state_probe LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT IS_ABSOLUTE "${UNITREE_SDK_SOURCE_DIR}" OR NOT EXISTS "${UNITREE_SDK_SOURCE_DIR}/include/unitree/idl/hg/LowState_.hpp")
  message(FATAL_ERROR "Provide an absolute UNITREE_SDK_SOURCE_DIR pointing to the pinned SDK")
endif()
find_package(Git REQUIRED)
execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
  WORKING_DIRECTORY "${UNITREE_SDK_SOURCE_DIR}"
  OUTPUT_VARIABLE H2_STATE_SDK_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE H2_STATE_GIT_RESULT)
if(NOT H2_STATE_GIT_RESULT EQUAL 0 OR NOT H2_STATE_SDK_HEAD STREQUAL "8e1b927586712edd6a4c29d50dd52dff1e163439")
  message(FATAL_ERROR "The passive probe requires the fixed Unitree SDK revision")
endif()
# The SDK ships its own Cyclone DDS. This target is deliberately not a ROS node.
set(BUILD_EXAMPLES OFF CACHE BOOL "Never build or run robot control examples" FORCE)
add_subdirectory("${UNITREE_SDK_SOURCE_DIR}" unitree_sdk2 EXCLUDE_FROM_ALL)
add_executable(h2_state_probe main.cpp)
target_compile_options(h2_state_probe PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(h2_state_probe PRIVATE unitree_sdk2)
set_target_properties(h2_state_probe PROPERTIES
  BUILD_RPATH "${UNITREE_SDK_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
  INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS h2_state_probe RUNTIME DESTINATION bin)
install(DIRECTORY "${UNITREE_SDK_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/"
  DESTINATION lib FILES_MATCHING PATTERN "libddsc*.so*")
