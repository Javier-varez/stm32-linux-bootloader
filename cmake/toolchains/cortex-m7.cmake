set(CMAKE_SYSTEM_NAME, Generic)
set(CMAKE_SYSTEM_PROCESSOR, arm)

set(triple armv7em-none-eabihf)
set(ARM_TARGET_CONFIG "--config armv7em_hard_fpv5_d16_nosys -mcpu=cortex-m7 -mfloat-abi=hard -mthumb")
set(CMAKE_C_COMPILER ${CMAKE_SOURCE_DIR}/.tools/llvm/bin/clang)
set(CMAKE_C_COMPILER_TARGET ${triple})
set(CMAKE_CXX_COMPILER ${CMAKE_SOURCE_DIR}/.tools/llvm/bin/clang++)
set(CMAKE_CXX_COMPILER_TARGET ${triple})

set(arm_flags "${ARM_TARGET_CONFIG} -fno-exceptions -fno-rtti -ffunction-sections -fdata-sections")
set(CMAKE_C_FLAGS "${arm_flags} -DARMV7_ARCH")
set(CMAKE_CXX_FLAGS "${arm_flags} -DARMV7_ARCH")

link_libraries(-lnosys -lc -lc++)

set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
