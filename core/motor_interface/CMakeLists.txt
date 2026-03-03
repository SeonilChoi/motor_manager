cmake_minimum_required(VERSION 3.16)
project(motor_interface VERSION 0.1.0 LANGUAGES CXX)

add_library(motor_interface INTERFACE)

target_include_directories(motor_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(motor_interface INTERFACE cxx_std_17)

add_library(motor_interface::motor_interface ALIAS motor_interface)
