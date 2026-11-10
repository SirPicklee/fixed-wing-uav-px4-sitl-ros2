cmake_minimum_required(VERSION 3.16)
project(fw_aero_gazebo VERSION 0.1.0 LANGUAGES CXX)

# Gazebo Harmonic (gz-sim 8) system plugin that applies the fw_aero
# aerodynamic model to a model link. All physics, frame conversions and
# PX4 command mapping live in fw_aero (unit-tested); this is a thin adapter.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(fw_aero REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(FixedWingAeroSystem SHARED
  src/FixedWingAeroSystem.cpp
)
target_link_libraries(FixedWingAeroSystem PRIVATE
  fw_aero::fw_aero
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
target_compile_options(FixedWingAeroSystem PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS FixedWingAeroSystem
  LIBRARY DESTINATION lib
)


install(DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)