# Head start to using Microsoft's Azure Kinect with OpenCV

When running the CMake project for the first time, enable the `INITIALIZE_K4A_SDK` option.
This will help initialize the [`SDK`](https://github.com/microsoft/Azure-Kinect-Sensor-SDK). Please keep in mind this project assumes you have all the  [`SDK`](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) dependencies.
If you don't have all the dependencies, the scripts directory may be a good place to start.

n.b., before going through the ceremonial CMake build process, make sure to enable the `INITIALIZE_K4A_SDK` option `(ln:29 CMakeLists.txt)`.

    option(INITIALIZE_K4A_SDK "Initialize the Kinect SDK project" ON) # default ON <- use for initializing Kinect SDK

To go through the examples conveniently, simply copy-paste each example into main.
