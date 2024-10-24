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

### Linker Bug
For some reason, STM32CubeMX has a bug in generating the linker script, and misses RAM in a few places

Ignore the project generated linker file, as I changed the CMakeLists.txt to link to the one in raven folder for both hubs
