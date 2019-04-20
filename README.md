
#### A Working copy of TrustZone (OPTEE 3.1.0) using Debian (Ubuntu) 64 bit FileSystem.

* First, burn the official Ubunut 64 bit image to micro sd card and see if RPi3 boots. Image Name/Link:

`ubuntu-18.04.2-preinstalled-server-arm64+raspi3.img.xz`

https://wiki.ubuntu.com/ARM/RaspberryPi


* In the host machine: 

1. Download toolchains for build:

```
cd /build
make -j2 toolchains
```

2. Now Make:

```
make -j `nproc`
```


See the details in OPTEE Guide: https://optee.readthedocs.io/building/gits/build.html#build

3. Run check `copy_to_sdcard.sh` for details of copying the compiled code to Raspberry Pi


##### Note: External interface say I2C/SPI etc not working due to the U-Boot version used in this repo. See the issue:
https://github.com/OP-TEE/optee_os/issues/2957
