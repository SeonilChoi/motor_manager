cmake_minimum_required(VERSION 3.16)
project(motor_manager_system VERSION 0.1.0 LANGUAGES CXX)

# Core interface (no sources)
add_subdirectory(core/motor_interface)

# Communications and hardware implementations
add_subdirectory(communications/ethercat)
add_subdirectory(hardware/minas)

# Main library (orchestrates masters, drivers, controllers)
add_subdirectory(motor_manager)
