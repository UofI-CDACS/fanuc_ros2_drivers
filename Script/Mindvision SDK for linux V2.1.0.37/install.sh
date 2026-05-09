#!/bin/bash

CURDIR=`pwd`
echo "Your current directory is $CURDIR. This is where the MVSDK software will be installed..."
A=`whoami`
B=`arch`

if [ $A != 'root' ]; then
   echo "You have to be root to run this script"
   echo "Fail !!!"
   exit 1;
fi


cp 88-mvusb.rules /etc/udev/rules.d/
cp 99-mvusb.rules /etc/udev/rules.d/

cp include/* /usr/include/
echo "Copy header files"

if [ $B == 'x86_64' ]; then
	cp lib/x64/libMVSDK.so /lib
	echo "Copy x64/libMVSDK.so to /lib"
elif [ $B == 'aarch64' ]; then
	cp lib/arm64/libMVSDK.so /lib
	echo "Copy arm64/libMVSDK.so to /lib"
elif [[ ${B:2} == '86' ]]; then
	cp lib/x86/libMVSDK.so /lib
	echo "Copy x86/libMVSDK.so to /lib"
else
	cp lib/arm/libMVSDK.so /lib
	echo "Copy arm/libMVSDK.so to /lib"
fi

echo "Successful"
echo "Please  restart system  now!!!"
