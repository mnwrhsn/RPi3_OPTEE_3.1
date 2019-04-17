#!/bin/sh


# run this inside Raspberry Pi (make sure to become root -- sudo su)

BUILD_MACHINE="mhasan@192.168.200.11"

BUILD_PATH_HOST="/home/mhasan/workspace/rt_security_io_tz/optee_examples/rt_io_sec/host/rt_io_sec_client"
OUT_PATH_HOST="/bin/"


BUILD_PATH_TA="/home/mhasan/workspace/rt_security_io_tz/optee_examples/rt_io_sec/ta/0f8a8fca-611f-11e9-8647-d663bd873d93.ta"
OUT_PATH_TA="/lib/optee_armtz/"

# execute as root
# [ `whoami` = root ] || exec su -c $0 root

# copy the client
sudo scp $BUILD_MACHINE:$BUILD_PATH_HOST $OUT_PATH_HOST

# copy the TA
sudo scp $BUILD_MACHINE:$BUILD_PATH_TA $OUT_PATH_TA

echo "Script finished!"
