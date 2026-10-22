# Cross-compiles libfpv for 64-bit Windows from Linux with MinGW-w64.
# The "-posix" compilers provide std::thread (winpthreads); everything is
# linked statically, so libfpv.dll has no runtime dependencies.
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(TOOLCHAIN_PREFIX x86_64-w64-mingw32)
find_program(CMAKE_C_COMPILER NAMES ${TOOLCHAIN_PREFIX}-gcc-posix ${TOOLCHAIN_PREFIX}-gcc REQUIRED)
find_program(CMAKE_CXX_COMPILER NAMES ${TOOLCHAIN_PREFIX}-g++-posix ${TOOLCHAIN_PREFIX}-g++ REQUIRED)
find_program(CMAKE_RC_COMPILER NAMES ${TOOLCHAIN_PREFIX}-windres)
set(CMAKE_FIND_ROOT_PATH /usr/${TOOLCHAIN_PREFIX})

# libfpv-PZ: declare strsep() for Betaflight's cli.c (see mingw-compat.h): without it the
# implicit int declaration cuts the 64-bit pointer it returns to 32 bits.
set(CMAKE_C_FLAGS_INIT "-include ${CMAKE_CURRENT_LIST_DIR}/mingw-compat.h")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Every binary links the GCC, C++ and winpthreads runtimes statically:
# without this, executables need libgcc_s_seh-1.dll, libstdc++-6.dll and
# libwinpthread-1.dll next to them (and fail to start silently otherwise).
set(CMAKE_EXE_LINKER_FLAGS_INIT "-static-libgcc -static-libstdc++ -static")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-static-libgcc -static-libstdc++ -static")
