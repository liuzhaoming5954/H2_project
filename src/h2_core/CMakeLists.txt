cmake_minimum_required(VERSION 3.16)
project(h2_core VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
find_package(Threads REQUIRED)
set(H2_CORE_WITH_AMENT FALSE)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  find_package(ament_cmake QUIET)
  if(ament_cmake_FOUND)
    set(H2_CORE_WITH_AMENT TRUE)
  endif()
endif()

add_library(h2_core src/motion_guard.cpp)
add_library(h2::core ALIAS h2_core)
target_compile_features(h2_core PUBLIC cxx_std_17)
target_include_directories(h2_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(h2_core PUBLIC Threads::Threads)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(h2_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS h2_core EXPORT h2_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../tests/core
      ${CMAKE_CURRENT_BINARY_DIR}/core_tests)
  endif()
endif()

if(H2_CORE_WITH_AMENT)
  ament_export_targets(h2_coreTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Threads)
  ament_package()
else()
  include(CMakePackageConfigHelpers)
  configure_package_config_file(cmake/h2_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/h2_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/h2_core)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/h2_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(EXPORT h2_coreTargets NAMESPACE h2_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/h2_core)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/h2_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/h2_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/h2_core)
endif()
