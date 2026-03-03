cmake_minimum_required(VERSION 3.16)
project(ethercat VERSION 0.1.0 LANGUAGES CXX)

add_library(ethercat
  src/ethercat_master.cpp
  src/ethercat_controller.cpp
)

target_include_directories(ethercat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ethercat PUBLIC motor_interface::motor_interface)

target_compile_features(ethercat PUBLIC cxx_std_17)

add_library(ethercat::ethercat ALIAS ethercat)
