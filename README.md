# Arduino nano 33 ble sense rev2 from scratch
The aim of this project is to develop hardware drivers for the Arduino nano33 ble sense rev 2 from scratch, using lightweight toolchain and build tools.
# Prerequisites
* Linux environement
* **arm-none-eabi-gcc** toolchain
# Compilation
```
make
```
# Debug
The debug and flash writing is based on a generic open source solution which is **Blackmagic probe firmware** written on an STM32F103C6T6.
The STM32F103C6T6 serves as a debug probe combined with gdb.
