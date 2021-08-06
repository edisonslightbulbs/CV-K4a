project(subtract)

# main project include paths
set(ROOT ${CMAKE_SOURCE_DIR})
set(SRC_DIR ${ROOT}/src)
set(EXT_DIR ${ROOT}/external)
set(LIBS_DIR ${ROOT}/libs)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# dependencies
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)

# after SDK initialization setup K4A (kinect SDK) paths
set(K4A_SDK ${EXT_DIR}/Azure-Kinect-Sensor-SDK)
set(K4A_LIBRARY ${K4A_SDK}/build)
set(K4A_VERSION ${K4A_SDK}/build/src/sdk/include)
set(K4A_INCLUDE ${K4A_SDK}/include)

# find include directories
set (INCLUDE_DIRS "")
file(GLOB_RECURSE HEADERS
    ${LIBS_DIR}/*.h
    )
foreach (HEADER ${HEADERS})
    get_filename_component(DIR ${HEADER} PATH)
    list (APPEND INCLUDE_DIRS ${DIR})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

# find src
file(GLOB_RECURSE LIBS_SRC
    ${LIBS_DIR}/*.cpp
    )

# add target
add_executable(subtract
    ${EXT_SRC}
    ${LIBS_SRC}
    subtract.cpp
    )

# target includes
target_include_directories(subtract PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${K4A_VERSION}
    ${K4A_INCLUDE}
    ${INCLUDE_DIRS}
    )

# link libraries
target_link_libraries(subtract
    ${OpenCV_LIBS}
    glog
    gflags
    ${K4A_LIBRARY}/bin/libk4a.so
    )
