#!/bin/bash

LOCAL_DIR=`pwd`
export PATH=$PATH:$LOCAL_DIR/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin
mkdir ../out
make O=../out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- msm_defconfig
make O=../out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- -j4

