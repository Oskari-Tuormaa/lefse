cmake_minimum_required(VERSION 3.20)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uart_echo)

target_sources(app PRIVATE src/main.cpp)
target_include_directories(app PRIVATE include)
