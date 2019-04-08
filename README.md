
Implementation of I/O-Peripheral Security for RTS

*1. Download toolchains for build:*

```
cd /build
make -j2 toolchains
```

*2. Now Make:*

```
make -j `nproc`
```


See the details in OPTEE Guide: https://optee.readthedocs.io/building/gits/build.html#build

*3. Run check `copy_to_sdcard.sh` for details of copying the compiled code to Raspberry Pi
