#!/usr/bin/env bash

# build_3dintact_toolkit.sh:
#     Builds the 3DINTACToolkit
#     cmake project using with -GNinja.
#
# author: Everett
# created: 2020-11-06 16:31
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))
echo "-- building 3DINTACToolkit from: $PROJECT_DIR"

# -- build directory
build="$PROJECT_DIR/build"

# -- clean old build files
if [ -d "$build" ]; then rm -Rf "$build"; fi

# -- build with CMake
mkdir "$build" && cd "$build" || return
cmake ..
make
cd - || return
