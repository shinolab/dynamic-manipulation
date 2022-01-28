cmake_minimum_required(VERSION 3.8)

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
set_target_properties(demo_manipulator PROPERTIES FOLDER "demos")


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
set_target_properties(demo_arf PROPERTIES FOLDER "demos")


add_executable(MobilityEvalTest MobilityEvalTest.cpp)
target_link_libraries(MobilityEvalTest 
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
    ThrustSearch
)
set_target_properties(MobilityEvalTest PROPERTIES FOLDER "test")

add_executable(WorkspaceEvalTest WorkspaceEvalTest.cpp)
target_link_libraries(WorkspaceEvalTest 
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(WorkspaceEvalTest PROPERTIES FOLDER "test")

add_executable(TrajectoryTest TrajectoryTest.cpp)
target_link_libraries(
    TrajectoryTest
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(TrajectoryTest PROPERTIES FOLDER "test")

add_executable(CircularEvalTest CircularEvalTest.cpp)
target_link_libraries(
    CircularEvalTest
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(CircularEvalTest PROPERTIES FOLDER "test")

add_executable(MuxGainTest MuxGainTest.cpp)
target_link_libraries(
    MuxGainTest
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(MuxGainTest PROPERTIES FOLDER "test")

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
set_target_properties(testVelTrans PROPERTIES FOLDER "test")

add_executable(demo_catch demo_catch.cpp)
target_link_libraries(
    demo_catch
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(demo_catch PROPERTIES FOLDER "demos")

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
    set_target_properties(ActionHandlerTest PROPERTIES FOLDER "test")

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
    set_target_properties(HandStateReaderTest PROPERTIES FOLDER "test")

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

    add_executable(
        demo_grasp
        demo_grasp.cpp
    )
    target_link_libraries(
        demo_grasp
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
    set_target_properties(demo_grasp PROPERTIES FOLDER "demos")

    add_executable(
        demo_hand_state
        demo_hand_state.cpp
    )
    target_link_libraries(
        demo_hand_state
        DynamicManipulation
        StereoTracker
        CameraDevice
        ImgProcUtil
        HandStateReader
        pcl_util
        pcl_grabber
        pcl_viewer
        ActionHandler
        DrawUtil
    )
    set_target_properties(demo_hand_state PROPERTIES FOLDER "demos")

endif()

add_executable(CreateSequenceTest CreateSequenceTest.cpp)
target_link_libraries(
    CreateSequenceTest
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(CreateSequenceTest PROPERTIES FOLDER "test")

add_executable(ComputeDutyTest ComputeDutyTest.cpp)
target_link_libraries(
    ComputeDutyTest
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(ComputeDutyTest PROPERTIES FOLDER "test")

add_executable(
    ThrustSearchTest
    ThrustSearchTest.cpp
)
target_link_libraries(
    ThrustSearchTest
    ThrustSearch
    StereoTracker
    CameraDevice
    ImgProcUtil
    DynamicManipulation
)
set_target_properties(ThrustSearchTest PROPERTIES FOLDER "test")

add_executable(
    FloatingObjectTest
    FloatingObjectTest.cpp
)
target_link_libraries(
    FloatingObjectTest
    DynamicManipulation   
)
set_target_properties(FloatingObjectTest PROPERTIES FOLDER "test")