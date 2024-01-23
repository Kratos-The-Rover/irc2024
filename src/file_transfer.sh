#!/usr/bin/env bash

source=$(who | awk '{print $1}')

echo $source

echo "the source is $source"

echo "enter Dest username"

read user



echo starting transfer....

scp /home/$source/Desktop/Captures $user@192.168.1.11:/home/$user/Desktop

echo starting transfer for panorama results

scp /home/$source/Desktop/panaorama_results $user@192.168.1.11:/home/$user/Desktop

echo transfer completed..

exit    