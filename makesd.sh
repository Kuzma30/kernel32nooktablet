#!/bin/sh
cp arch/arm/boot/zImage ~/bootimg-tools/boot.img-kernel
cd ~/bootimg-tools/
./packboot
mount /mnt/sd1/
cp boot_new.img /mnt/sd1/boot.img
umount /mnt/sd1/