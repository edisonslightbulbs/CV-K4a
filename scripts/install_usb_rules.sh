#!/usr/bin/env bash

# install_usb_rules.sh
#     Installs the Azure-Kinects' USB driver from the driver directory.
#
# author: Everett
# created: 2020-11-06 13:56
# Github: https://github.com/antiqueeverett/

sudo cp "$PWD/99-k4a.rules" '/etc/udev/rules.d/'
