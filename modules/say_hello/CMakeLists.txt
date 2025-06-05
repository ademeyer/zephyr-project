# Check if SAY_HELLO is enabled in Kconfig
if(CONFIG_SAY_HELLO)
  # Add current directory to include path
  zephyr_include_directories(.)
  
  # Add source file to build
  zephyr_library_sources(say_hello.c)
endif()
