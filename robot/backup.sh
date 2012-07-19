#!/bin/sh

dirname=/mnt/zip/robot1/`date +"%m-%d-%Y_%H_%M"`

if [ -f $dirname ]
then
	echo $dirname already exists\!
	exit 1
fi

echo Backing up to $dirname

mount /mnt/zip
mkdir $dirname
cp -r ~profesor/rtems/tools/robot1/* $dirname
umount /mnt/zip

