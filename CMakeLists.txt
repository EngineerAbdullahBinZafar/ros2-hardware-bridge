cmake_minimum_required(VERSION 3.16)
project(ros2_hardware_bridge LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(ros2_bridge_firmware STATIC
    firmware/src/ros2_bridge_fsm.c
    firmware/src/stm32_uart_dma.c
)
target_include_directories(ros2_bridge_firmware PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/firmware/include
)
target_compile_options(ros2_bridge_firmware PRIVATE -Wall -Wextra -Werror)

enable_testing()

add_executable(test_e2e_c firmware/tests/test_e2e_c.c)
target_link_libraries(test_e2e_c PRIVATE ros2_bridge_firmware)
target_compile_options(test_e2e_c PRIVATE -Wall -Wextra -Werror)
add_test(NAME firmware_e2e_c_tests COMMAND test_e2e_c)

add_executable(test_serial_e2e_cpp ros2_cpp/tests/test_serial_e2e_cpp.cpp)
target_include_directories(test_serial_e2e_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_cpp/include
)
target_link_libraries(test_serial_e2e_cpp PRIVATE ros2_bridge_firmware)
target_compile_options(test_serial_e2e_cpp PRIVATE -Wall -Wextra -Werror)
add_test(NAME host_serial_e2e_cpp_tests COMMAND test_serial_e2e_cpp)
