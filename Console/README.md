## Compiling the firmware

```bash
# clone Riot OS
git clone -b 2018.04 --depth 1 https://github.com/RIOT-OS/RIOT.git

cd RIOT
git apply ../patch

# where Console application locates
cd examples/default
make WERROR=0 # it produces bin/frdm-k64f/default.elf elf file
```
