#!/usr/bin/env bash

# install_depthengine.sh:
#     Installs the depth engine binary from ./bin directory.
#
# author: Everett
# created: 2020-11-06 13:54
# Github: https://github.com/antiqueeverett/

sudo cp "$PWD/libdepthengine.so.2.0" '/usr/lib/'
sudo ldconfig
