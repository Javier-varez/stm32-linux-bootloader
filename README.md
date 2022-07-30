# STM32 F7 Discovery Linux Bootloader

```
./bootstrap.sh
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/cortex-m7.cmake -DCMAKE_SYSTEM_NAME=Generic
cmake --build build -- run
```

