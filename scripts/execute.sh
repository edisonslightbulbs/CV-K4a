#!/usr/bin/env bash

# execute_3dintact.sh:
#   executes toolkit
#
# author: Everett
# created: 2021-04-15 18:59
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))
echo "-- executing project"

# -- BIN directory
cd "$PROJECT_DIR" || return

mkdir -p ./output
rm -rf ./output/*.*
./build/bin/CVKinect --logtostderr=1
