# Check if SAI_STM32 is enabled in Kconfig
if(CONFIG_SAI_STM32)
  add_compile_options(-Wno-unused-function -Wmaybe-uninitialized)
  zephyr_library()

  # Add current directory to include path
  zephyr_include_directories(.)

  # Add source file to build
  # zephyr_library_sources(i2s_common.c)
  zephyr_library_sources(stm32_ll_sai.c)
  zephyr_library_sources(i2s_sai_ll_stm32.c)
endif()
