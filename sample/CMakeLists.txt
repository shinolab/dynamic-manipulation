cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
cmake_policy(SET CMP0074 NEW)

# Eigen3
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${PROJECT_SOURCE_DIR}/deps/autd3-library-software/client/include )
include_directories(${PROJECT_SOURCE_DIR}/inc)

# autd
include_directories(${PROJECT_SOURCE_DIR}/deps/autd3-library-software/client/lib)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find Kinect SDK v2
find_package( KinectSDK2 REQUIRED )
include_directories( ${KinectSDK2_INCLUDE_DIRS} )
link_directories( ${KinectSDK2_LIBRARY_DIRS} )

# Find Ximea
find_package(XIMEA REQUIRED)
include_directories(${XIMEA_INCLUDE_DIR})

add_executable(demo_manipulator demo_manipulator.cpp)
target_link_libraries(
    demo_manipulator
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

#add_executable(simple simple.cpp)
#target_link_libraries(simple DynamicManipulation)

#add_executable(pipeline pipeline.cpp)
#include_directories(../inc)
#target_link_libraries(pipeline DynamicManipulation)

#add_executable(projection projection.cpp)
#include_directories(../inc)
#target_link_libraries(projection DynamicManipulation Projector)

#add_executable(sequential sequential.cpp)
#include_directories(../inc)
#target_link_libraries(sequential DynamicManipulation)

#add_executable(impactor impactor.cpp ../inc/rootFindingPoly.hpp)
#target_link_libraries(impactor DynamicManipulation)

#add_executable(single_sided single_sided.cpp)
#target_link_libraries(single_sided DynamicManipulation)

#add_executable(ss_trajectory single_sided_with_trajectory.cpp)
#target_link_libraries(ss_trajectory DynamicManipulation)

#add_executable(demo_proactive demo_proactive.cpp)
#target_link_libraries(demo_proactive DynamicManipulation)

#add_executable(demo_guidance demo_guidance.cpp init_uist.hpp)
#target_link_libraries(demo_guidance DynamicManipulation Projector)

#add_executable(demo_notification demo_notification.cpp init_uist.hpp)
#target_link_libraries(demo_notification DynamicManipulation Projector)

#add_executable(demo_avatar demo_avatar.cpp init_uist.hpp)
#target_link_libraries(demo_avatar DynamicManipulation Projector)

#add_executable(demo_instruction demo_instruction.cpp init_uist.hpp)
#target_link_libraries(demo_instruction DynamicManipulation Projector)

#add_executable(demo_volleyball demo_volleyball.cpp init_uist.hpp)
#target_link_libraries(demo_volleyball DynamicManipulation Projector)

#add_executable(demo_innovative demo_innovative.cpp init_uist.hpp)
#target_link_libraries(demo_innovative DynamicManipulation)

#add_executable(haptic_icon haptic_icon.hpp haptic_icon.cpp)
#target_link_libraries(haptic_icon DynamicManipulation StereoTracker CameraDevice ImgProcUtil)

add_library(MonitorInfoManager STATIC MonitorInfoManager.hpp MonitorInfoManager.cpp)

add_library(DrawUtil DrawUtil.hpp DrawUtil.cpp)
target_link_libraries(DrawUtil ${OpenCV_LIBS})

# add_executable(demo_sa2020_mobility
#     demo_sa2020_mobility.cpp
#     haptic_icon.hpp
# )
# target_link_libraries(demo_sa2020_mobility
#     StereoTracker
#     CameraDevice
#     ImgProcUtil
#     Strategies
#     DynamicManipulation
# )

# add_executable(demo_arf
#     demo_arf.cpp
#     haptic_icon.hpp
# )
# target_link_libraries(demo_arf
#     StereoTracker
#     CameraDevice
#     ImgProcUtil
#     Strategies
#     DynamicManipulation
# )

# add_executable(test_findDuty test_findDuty.cpp)
# target_link_libraries(test_findDuty
#     StereoTracker
#     CameraDevice
#     ImgProcUtil
#     DynamicManipulation
# )

add_executable(MobilityEvalTest MobilityEvalTest.cpp)
target_link_libraries(MobilityEvalTest DynamicManipulation)
