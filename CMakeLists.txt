cmake_minimum_required(VERSION 3.11)
project(cv-k4a)

# optimization for dev environments
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# project version
set(MAJOR 0)
set(MINOR 1)
set(REMAP_VERSION ${MAJOR}.${MINOR})

# flags
set(CMAKE_CXX_FLAGS "-pthread")

# set cmake modules path
list(APPEND CMAKE_MODULE_PATH "$ENV{CMAKE_MODULE_PATH}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# set path variables
set(PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})

set(BIN_DIR ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BIN_DIR})

option(OPTIMIZE_PROJECT "Tidy up and optimize source code" OFF) # default OFF
if(OPTIMIZE_PROJECT)
    if(UNIX AND NOT APPLE)
        set(CMAKE_CXX_CLANG_TIDY
            clang-tidy -p pwd
            --checks=-*,readability-*,clang-analyzer-*,performance-*)
    endif()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# dependencies
find_package(Git REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)

option(BUILD_EXAMPLES "Build example" OFF) # default ON
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# project paths
set(SRC_DIR ${PROJECT_DIR}/src)
set(EXT_DIR ${PROJECT_DIR}/external)
set(LIBS_DIR ${PROJECT_DIR}/libs)

option(INITIALIZE_K4A_SDK "Initialize the kinect SDK project" OFF) # default ON
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    if(INITIALIZE_K4A_SDK)
        execute_process(
            COMMAND /bin/bash ./scripts/initialize_azure_kinect_sensor_sdk.sh -j 12
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
    endif()
endif()

# after SDK initialization setup K4A (kinect SDK) paths
set(K4A_SDK ${EXT_DIR}/Azure-Kinect-Sensor-SDK)
set(K4A_LIBRARY ${K4A_SDK}/build)
set(K4A_VERSION ${K4A_SDK}/build/src/sdk/include)
set(K4A_INCLUDE ${K4A_SDK}/include)

# find header directories
set (INCLUDE_DIRS "")
file(GLOB_RECURSE HEADERS
    ${LIBS_DIR}/*.h
    )
foreach (HEADER ${HEADERS})
    get_filename_component(DIR ${HEADER} PATH)
    list (APPEND INCLUDE_DIRS ${DIR})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

# find src files
file(GLOB_RECURSE LIBS_SRC
    ${LIBS_DIR}/*.cpp
    )

# add executable
add_executable(cv-k4a
    ${EXT_SRC}
    ${LIBS_SRC}
    ${SRC_DIR}/main.cpp
    )

# include directories
target_include_directories(cv-k4a PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${K4A_VERSION}
    ${K4A_INCLUDE}
    ${INCLUDE_DIRS}
    )

# link libraries
target_link_libraries(cv-k4a
    ${OpenCV_LIBS}
    glog
    gflags
    ${K4A_LIBRARY}/bin/libk4a.so
    )

option(EXECUTE_TARGET "Execute post build" ON) # default OFF
if(EXECUTE_TARGET)
    add_custom_command(TARGET cv-k4a
        POST_BUILD
        COMMAND ./scripts/execute.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
endif()
