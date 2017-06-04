#!/bin/sh

#Architecture
export ARCH=arm

#u-boot machine
UBOOT_MACHINE=am57xx_evm_config

#Points to the root of the TI SDK
export TI_SDK_PATH=/home/adi/work/am57xx/ti-processor-sdk-linux-am57xx-evm-03.03.00.04

#root of the target file system for installing applications
export DESTDIR=/home/adi/work/am57xx/ti-processor-sdk-linux-am57xx-evm-03.03.00.04/targetNFS

#Points to the root of the Linux libraries and headers matching the
#demo file system.
export LINUX_DEVKIT_PATH=$TI_SDK_PATH/linux-devkit

#Cross compiler prefix
export CROSS_COMPILE=$LINUX_DEVKIT_PATH/sysroots/x86_64-arago-linux/usr/bin/arm-linux-gnueabihf-

make $1
