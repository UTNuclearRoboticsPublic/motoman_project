#!/bin/sh

echo "Start installing sudo"
apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/
echo "Finish installing sudo"

echo "Start installing PCL"
sudo apt-get install -y software-properties-common python-software-properties; sudo apt-add-repository -y ppa:v-launchpad-jochen-sprickerhof-de/pcl; sudo apt-get update -qq; sudo apt-get install -y libpcl-all;
echo "Finish installing PCL"
