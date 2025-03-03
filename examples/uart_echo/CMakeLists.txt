cmake_minimum_required(VERSION 3.20)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uart_echo)

target_sources(app PRIVATE src/main.cpp)
target_include_directories(app PRIVATE include)

if(CONFIG_UART_ASYNC_API AND CONFIG_UART_INTERRUPT_DRIVEN)
    message(FATAL_ERROR "Both async and interrupt UART API's chosen at once. Please choose either one or none.")
elseif(CONFIG_UART_ASYNC_API)
    target_sources(app PRIVATE ./src/uart_async.cpp)
elseif(CONFIG_UART_INTERRUPT_DRIVEN)
    target_sources(app PRIVATE ./src/uart_interrupt.cpp)
else()
    target_sources(app PRIVATE ./src/uart_polling.cpp)
endif()
