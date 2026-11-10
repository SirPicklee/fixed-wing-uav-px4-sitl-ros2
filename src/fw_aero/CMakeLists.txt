cmake_minimum_required(VERSION 3.16)
project(fw_aero VERSION 0.1.0 LANGUAGES CXX)

# Pure C++ aerodynamic model of a TB2-class fixed-wing UAV.
# No ROS or Gazebo dependency: the Gazebo plugin is a thin layer on top.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# --- Library -----------------------------------------------------------------
add_library(fw_aero
  src/atmosphere.cpp
  src/aerodynamics.cpp
)
target_include_directories(fw_aero PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fw_aero PUBLIC Eigen3::Eigen)
target_compile_options(fw_aero PRIVATE -Wall -Wextra -Wpedantic)
# Position-independent code: the library is later linked into a Gazebo
# plugin, which is a shared object.
set_target_properties(fw_aero PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- Tests -------------------------------------------------------------------
option(BUILD_TESTING "Build unit tests" ON)
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(fw_aero_tests
    test/test_atmosphere.cpp
    test/test_aerodynamics.cpp
    test/test_python_parity.cpp
  )
  target_link_libraries(fw_aero_tests PRIVATE fw_aero GTest::gtest_main)
  target_compile_definitions(fw_aero_tests PRIVATE
   FW_AERO_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_compile_options(fw_aero_tests PRIVATE -Wall -Wextra -Wpedantic)

  include(GoogleTest)
  gtest_discover_tests(fw_aero_tests)
endif()