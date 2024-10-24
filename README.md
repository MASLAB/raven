# raven
Robot Hardware Controller

## Pcb
Color pallet works best with KiCad wDark theme

## Firmware
Built using cmake and all dependencies exist locally

Install build tools if you need to
```
sudo apt install ninja-build cmake gcc-arm-none-eabi
```

### Compiling
Compile from either sensor-hub or motor-hub folder
```
cmake . --preset=Release
cd Release
ninja
```

### Linker Bug
For some reason, STM32CubeMX has a bug in generating the linker script, and misses RAM in a few places

Ignore the project generated linker file, as I changed the CMakeLists.txt to link to the one in firmware folder for both hubs
