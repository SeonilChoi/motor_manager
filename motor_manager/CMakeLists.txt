cmake_minimum_required(VERSION 3.16)
project(motor_manager VERSION 0.1.0 LANGUAGES CXX)

find_package(yaml-cpp REQUIRED)

add_library(motor_manager src/motor_manager.cpp)

target_include_directories(motor_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(motor_manager
  PUBLIC  motor_interface::motor_interface
  PRIVATE ethercat::ethercat
  PRIVATE minas::minas
  PRIVATE yaml-cpp
)

target_compile_features(motor_manager PUBLIC cxx_std_17)

add_library(motor_manager::motor_manager ALIAS motor_manager)
