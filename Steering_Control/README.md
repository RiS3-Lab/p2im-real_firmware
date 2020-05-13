## Compiling the firmware

```
../deps/arduino-cli/arduino-cli compile -b arduino:sam:arduino_due_x_dbg *.ino --build-path `realpath ./build` -o firmware
```
firmware.elf is the binary file generated.
