cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)
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

add_executable(demo_arf
    demo_arf.cpp
    haptic_icon.hpp
)
target_link_libraries(demo_arf
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

# add_executable(test_findDuty test_findDuty.cpp)
# target_link_libraries(test_findDuty
#     StereoTracker
#     CameraDevice
#     ImgProcUtil
#     DynamicManipulation
# )

add_executable(MobilityEvalTest MobilityEvalTest.cpp)
target_link_libraries(MobilityEvalTest 
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

add_executable(WorkspaceEvalTest WorkspaceEvalTest.cpp)
target_link_libraries(WorkspaceEvalTest 
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

add_executable(CircularEvalTest CircularEvalTest.cpp)
target_link_libraries(
    CircularEvalTest
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

add_executable(Stationary stationary.cpp)
target_link_libraries(Stationary
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

add_library(
    vibrator
    vibrator.hpp
    vibrator.cpp
    )
target_link_libraries(
    vibrator
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

add_executable(testVibration testVibration.cpp)
target_link_libraries(
    testVibration
    vibrator
)

add_executable(testVibrationSimple testVibrationSimple.cpp)
target_link_libraries(testVibrationSimple
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

add_executable(testVibrationJnd testVibrationJnd.cpp)
target_link_libraries(
    testVibrationJnd
    vibrator
)

add_executable(
    testVelTrans
    testVelTrans.cpp
    )
target_link_libraries(
    testVelTrans
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)

option(BUILD_PCL_APPS "build examples that use pcl" ON)
#option(BUILD_AZURE_KINECT_APPS "requires GPU better than GeForce 1070 or better." ON)
if(BUILD_PCL_APPS)
    find_package(realsense2 REQUIRED)
    include_directories(${realsense2_INCLUDE_DIR})

    find_package(OpenGL REQUIRED)
    include_directories(${OPENGL_INCLUDE_DIR})

    find_package(GLEW REQUIRED)
    include_directories(${GLEW_INCLUDE_DIRS})

    find_package(GLFW3 REQUIRED)
    include_directories(${GLFW3_INCLUDE_DIR})

    find_package(PCL REQUIRED)
    include_directories(${PCL_INCLUDE_DIR})
    add_definitions(${PCL_DEFINITIONS})

    add_library(pcl_util pcl_util.hpp pcl_util.cpp)
    target_link_libraries(
        pcl_util
        ${PCL_LIBRARIES}
        DynamicManipulation
    )
    add_library(pcl_viewer pcl_viewer.hpp pcl_viewer.cpp)
    target_link_libraries(
        pcl_viewer
        pcl_util
        DynamicManipulation
        ${OPENGL_LIBRARIES}
        ${PCL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${GLFW3_LIBRARY}
    )

    include_directories(${PCL_INCLUDE_DIR})
    add_definitions(${PCL_DEFINITIONS})
    add_library(pcl_grabber pcl_grabber.cpp pcl_grabber.hpp)
    target_link_libraries(
        pcl_grabber
        pcl_util
        ${realsense2_LIBRARY}
        ${PCL_LIBRARIES}
    )

    add_executable(pcl_viewer_test pcl_viewer_test.cpp)
    target_link_libraries(
        pcl_viewer_test
        pcl_viewer
        pcl_grabber
        ${realsense2_LIBRARY}
    )

    add_library(ActionHandler
        ActionHandler.hpp
        ActionHandler.cpp
    )
    add_executable(ActionHandlerTest
        ActionHandlerTest.cpp
    )
    target_link_libraries(ActionHandlerTest
        ActionHandler
    )

    add_library(HandStateReader
        HandStateReader.hpp
        HandStateReader.cpp
    )    
    target_link_libraries(
        HandStateReader
        pcl_util
        ${PCL_LIBRARIES}
        pcl_grabber
        DynamicManipulation
    )

    add_executable(HandStateReaderTest
        HandStateReaderTest.cpp
    )
    target_link_libraries(
        HandStateReaderTest
        DynamicManipulation
        StereoTracker
        CameraDevice
        ImgProcUtil
        HandStateReader
        pcl_util
        pcl_grabber
        pcl_viewer
    )

    add_executable(
        graspable
        graspable.cpp
    )
    target_link_libraries(
        graspable
        DynamicManipulation
        StereoTracker
        CameraDevice
        ImgProcUtil
        HandStateReader
        pcl_util
        pcl_grabber
        pcl_viewer
        ActionHandler
    )

endif()