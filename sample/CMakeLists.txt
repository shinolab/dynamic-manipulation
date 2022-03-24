cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# Eigen3
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${PROJECT_SOURCE_DIR}/deps/autd3-library-software/client/include )
include_directories(${PROJECT_SOURCE_DIR}/inc)

# autd
include_directories(${PROJECT_SOURCE_DIR}/deps/autd3-library-software/client/lib)

# OpenCV
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find Kinect SDK v2
find_package(KinectSDK2)
if(KINECTSDK2_FOUND)
    include_directories( ${KinectSDK2_INCLUDE_DIRS} )
    link_directories( ${KinectSDK2_LIBRARY_DIRS} )
endif()
# Find Ximea
find_package(XIMEA)
if(XIMEA_FOUND)
    include_directories(${XIMEA_INCLUDE_DIR})
endif()

add_executable(demo_manipulator demo_manipulator.cpp)
target_link_libraries(
    demo_manipulator
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(demo_manipulator PROPERTIES FOLDER "demos")

add_executable(demo_catch demo_catch.cpp)
target_link_libraries(
    demo_catch
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(demo_catch PROPERTIES FOLDER "demos")