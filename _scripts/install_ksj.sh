#!/bin/bash

case $(uname -m) in
    x86_64)  
        CPU_ARCH="x64"
    ;;
    aarch64) 
        CPU_ARCH="arm64"
    ;;
esac

wget https://offical-account-bs.oss-cn-beijing.aliyuncs.com/gitee/KSJ.tar
sudo tar -xvf KSJ.tar -C /opt
sudo ln -s /opt/KSJ/lib/${CPU_ARCH} /opt/KSJ/lib64
rm -rf KSJ.tar

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/KSJ/lib64' >> ~/.zshrc

ksj_udev_text='SUBSYSTEM=="usb",ATTRS{idVendor}=="0816",ATTRS{idProduct}=="1d15",GROUP="user",MODE="0777"'
echo -e ${ksj_udev_text} | sudo tee /etc/udev/rules.d/80-ksj-usb.rules