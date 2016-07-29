#!/usr/bin/env bash

# List apt-get dependencies here
APT_GET_DEPS=(
  ladspa-sdk
  tap-plugins
  libsox-fmt-all
  python-pygame
)
PACKAGE_DIR=$(dirname "$0")

# Loop each dependency checking if already satisfied
for i in "${APT_GET_DEPS[@]}"
do
  if [ $(dpkg-query -W -f='${Status}' $i 2>/dev/null | grep -c "ok installed") -eq 0 ];
  then 
    echo "Installing apt-get dependency $i"
    sudo apt-get install -y $i
  fi
done

# Install pip dependencies from requirements.txt file
echo "Installing pip dependencies in requirements.txt"
pip install -r ${PACKAGE_DIR}/requirements.txt
