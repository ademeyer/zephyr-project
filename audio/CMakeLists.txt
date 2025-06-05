cmake_minimum_required(VERSION 3.21.0)

set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../modules/say_hello")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(randomgenerator VERSION 1.0 LANGUAGES C DESCRIPTION "Testing random number generator on
stm32f7_disco")

target_sources(app PRIVATE src/main.c)
