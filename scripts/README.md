## Walk through to quickly get up & running with the Azure-Kinect-Sensor-SDK submodule

##### 1. The depth engine

A dated [Depth Engine](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/depthengine.md) binary is provided and can be installed by running the `install_depthengine.sh` helper script. Please follow the hyperlink for the official step-by-step on how to get the most up-to-date depth engine.

##### 2. The USB rules

For convenience, the USB rules can be installed by running the `install_usb_rules.sh` helper script in this directory.

##### 3. Building the submodule project

__Iff__  all dependencies have been installed, build the [Azure-Kinect-Sensor-SDK project](../external/submodules/Azure-Kinect-Sensor-SDK) project by running the `build_kinect_sdk.sh` script in this directory.