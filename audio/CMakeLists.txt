cmake_minimum_required(VERSION 3.21.0)

set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../modules/stm32f746_disco_sai")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(audio VERSION 1.0)

target_sources(app PRIVATE src/main.c)
