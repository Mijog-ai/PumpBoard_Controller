# groups.cmake

# group USER
add_library(Group_USER OBJECT
  "${SOLUTION_ROOT}/main.c"
)
target_include_directories(Group_USER PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_USER PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_USER_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_USER_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_USER PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_USER PUBLIC
  Group_USER_ABSTRACTIONS
)

# group CORE
add_library(Group_CORE OBJECT
  "${SOLUTION_ROOT}/../CORE/startup_stm32f40_41xxx.s"
  "${SOLUTION_ROOT}/../CORE/system_stm32f4xx.c"
)
target_include_directories(Group_CORE PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_CORE PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_CORE_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_CORE_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_CORE PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_CORE PUBLIC
  Group_CORE_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  __MICROLIB
  STM32F407xx
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/../CORE/startup_stm32f40_41xxx.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)
set_source_files_properties("${SOLUTION_ROOT}/../CORE/startup_stm32f40_41xxx.s" PROPERTIES
  COMPILE_OPTIONS "-masm=auto"
)

# group FWLIB
add_library(Group_FWLIB OBJECT
  "${SOLUTION_ROOT}/../FWLIB/src/misc.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_adc.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_can.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_crc.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_cryp.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_cryp_aes.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_cryp_des.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_cryp_tdes.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_dac.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_dbgmcu.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_dcmi.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_dma2d.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_dma.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_exti.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_flash.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_flash_ramfunc.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_fsmc.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_hash_md5.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_gpio.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_hash.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_hash_sha1.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_i2c.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_iwdg.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_ltdc.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_pwr.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_rcc.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_rng.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_rtc.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_sai.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_sdio.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_spi.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_syscfg.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_tim.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_usart.c"
  "${SOLUTION_ROOT}/../FWLIB/src/stm32f4xx_wwdg.c"
)
target_include_directories(Group_FWLIB PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_FWLIB PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_FWLIB_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_FWLIB_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_FWLIB PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_FWLIB PUBLIC
  Group_FWLIB_ABSTRACTIONS
)

# group HardWare_Init
add_library(Group_HardWare_Init OBJECT
  "${SOLUTION_ROOT}/../HardWare_Init/Src/Hw_SPI.c"
  "${SOLUTION_ROOT}/../HardWare_Init/Src/Hw_ADC.c"
  "${SOLUTION_ROOT}/../HardWare_Init/Src/Hw_GPIO.c"
  "${SOLUTION_ROOT}/../HardWare_Init/Src/Hw_TIM.c"
  "${SOLUTION_ROOT}/../HardWare_Init/Src/AD7689.c"
  "${SOLUTION_ROOT}/../HardWare_Init/Src/Hw_CAN.c"
)
target_include_directories(Group_HardWare_Init PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_HardWare_Init PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_HardWare_Init_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_HardWare_Init_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_HardWare_Init PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_HardWare_Init PUBLIC
  Group_HardWare_Init_ABSTRACTIONS
)

# group Function
add_library(Group_Function OBJECT
  "${SOLUTION_ROOT}/../Function/algorithm.c"
  "${SOLUTION_ROOT}/../Function/analog_input.c"
  "${SOLUTION_ROOT}/../Function/application.c"
  "${SOLUTION_ROOT}/../Function/flash.c"
  "${SOLUTION_ROOT}/../Function/stm32f4xx_it.c"
  "${SOLUTION_ROOT}/../Function/Task.c"
  "${SOLUTION_ROOT}/../Function/CheckTable.c"
  "${SOLUTION_ROOT}/../Function/limit.c"
  "${SOLUTION_ROOT}/../Function/Diagnose.c"
  "${SOLUTION_ROOT}/../Function/can_protocol.c"
)
target_include_directories(Group_Function PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Function PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Function_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Function_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Function PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Function PUBLIC
  Group_Function_ABSTRACTIONS
)

# group LIB
add_library(Group_LIB INTERFACE)
target_include_directories(Group_LIB INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_LIB INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_LIB_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_LIB_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_link_libraries(Group_LIB INTERFACE
  ${SOLUTION_ROOT}/../LIB/Deadzone_Current.lib
  ${SOLUTION_ROOT}/../LIB/PID_AREA.lib
)
