set(CMAKE_SYSTEM_NAME, Generic)
set(CMAKE_SYSTEM_PROCESSOR, arm)

set(ARM_TARGET_CONFIG "--config armv7m_soft_nofp.cfg")
set(CMAKE_C_COMPILER ${CMAKE_SOURCE_DIR}/.tools/llvm/bin/clang)
set(CMAKE_CXX_COMPILER ${CMAKE_SOURCE_DIR}/.tools/llvm/bin/clang++)

set(arm_flags "${ARM_TARGET_CONFIG} -ffunction-sections -fdata-sections")
set(CMAKE_C_FLAGS "${arm_flags} -DARMV7_ARCH")
set(CMAKE_CXX_FLAGS "${arm_flags} -DARMV7_ARCH")

link_libraries(-lc -lc++)

set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
