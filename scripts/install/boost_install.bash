#!/bin/sh
set -e  # exit on first error

echo "Installing Boost ..."
sudo apt-get install -qq libboost-* -y
