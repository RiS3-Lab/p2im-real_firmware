This is the repo for all real-world firmware fuzz-tested in [P<sup>2</sup>IM paper](https://www.usenix.org/conference/usenixsecurity20/presentation/feng).


## Fuzzing firmware with P<sup>2</sup>IM paper
Please follow the instructions [here](https://github.com/RiS3-Lab/p2im#fuzzing). You can either use the pre-compiled firmware binary in [`binary/`](binary/), or compile the firmware by yourself.


## Compiling firmware
Setup the compiler toolchain following these [instructions](https://github.com/RiS3-Lab/p2im#gnu-arm-embedded-toolchain).

For Arduino-based firmware, you need to set up the Arduino development environment:  
1) Download the [Arduino core](https://drive.google.com/file/d/13FO7wVcyBzSKz19MirNG6V7RgrgGietP/view?usp=sharing), and untar it to ~/Arduino

2) Download the [library](https://drive.google.com/file/d/1YTkcOAXTXF4obIdlH_Cd-sOcAt9OqwgO/view?usp=sharing) that firmware rely on, and untar it to ~/.arduino15

Now you are ready to compile firmware. Please follow the instructions in `README.md` under each directory.


## aflCall
All firmware invoke `aflCall`, as explained [here](https://github.com/RiS3-Lab/p2im/blob/master/docs/prep_fw_for_fuzzing.md). In QEMU, `aflCall` is intercepted and does not change firmware state. However, it may crash the firmware on real device. To disable `aflCall`, simply set `noHyperCall` variable to 1 on source code, or replace all `svc $0x3f` instructions by `NOP` on binary.


## TODO
I am still cleaning `CNC` and `Soldering_Iron` firmware.
