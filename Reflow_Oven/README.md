## Compiling the firmware

```
../deps/arduino-cli/arduino-cli compile -b STM32:stm32:Nucleo_64:pnum=NUCLEO_F103RB,upload_method=MassStorage,usb=none,opt=ogstd *.ino --build-path `realpath ./build` -o firmware
```
firmware.elf is the binary file generated.
