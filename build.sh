#!/bin/bash

# Copyright Joseph Riches <josephriches@gmail.com> 2014
# Based on build scripts by cybojenix <anthonydking@gmail.com> and DooMLoRD(@github)
# Copy, modify, or distribute as you will under the DWYWWT (do whatever you want with this) licence

if [ -z $target ]; then
    echo "choose your target device"
    echo "1) JLO (J)"
    echo "2) Mesona (Miro)"
    echo "3) Nanhu (E)"
    echo "4) Tapioca (Tipo)"
    read -p "1/2/3/4: " choice
    case "$choice" in
        1 ) export target=JLO ; export defconfig=cm_tamsui_jlo_defconfig;;
        2 ) export target=Mesona ; export defconfig=cm_tamsui_mes_defconfig;;
        3 ) export target=Nanhu ; export defconfig=cm_tamsui_nan_defconfig;;
        4 ) export target=Tapioca ; export defconfig=cm_tamsui_tap_defconfig;;
        * ) echo "invalid choice"; sleep 2 ; $0;;
    esac
fi # [ -z $target ]

export COMPILER_DIR=~/cm-11.0/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7/bin
export ARCH=arm
export CROSS_COMPILE=${COMPILER_DIR}/arm-linux-androideabi-

if [ -z "$clean" ]; then
    read -p "do make clean mrproper?(y/n)" clean
fi # [ -z "$clean" ]
case "$clean" in
    y|Y ) echo "cleaning..."; make clean mrproper;;
    n|N ) echo "continuing...";;
    * ) echo "invalid option"; sleep 2 ; build.sh;;
esac

echo "building the kernel"

make $defconfig
make -j `cat /proc/cpuinfo | grep "^processor" | wc -l` "$@"

if [ -f arch/arm/boot/zImage ]; then

	mkdir -p OUT
	rm OUT/zImage
	cp arch/arm/boot/zImage OUT/zImage

fi # [ -f arch/arm/boot/zImage ]

export MKELF_PY=~/cm-11.0/device/sony/tamsui-common/tools/mkelf.py







