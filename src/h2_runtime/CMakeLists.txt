cmake_minimum_required(VERSION 3.16)
project(h2_runtime)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(h2_interfaces REQUIRED)
find_package(h2_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(motion_guard src/motion_guard.cpp)
target_compile_features(motion_guard PRIVATE cxx_std_17)
ament_target_dependencies(motion_guard rclcpp h2_interfaces geometry_msgs)
target_link_libraries(motion_guard h2_core::h2_core)

add_executable(mock_platform src/mock_platform.cpp)
target_compile_features(mock_platform PRIVATE cxx_std_17)
ament_target_dependencies(mock_platform rclcpp h2_interfaces geometry_msgs nav_msgs tf2_ros)

install(TARGETS motion_guard mock_platform DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(runtime_contract test/test_runtime_contract.py TIMEOUT 45)
endif()

ament_package()
