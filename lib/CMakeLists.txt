cmake_minimum_required(VERSION 3.8.2)

set(CMAKE_CXX_STANDARD 17)

cmake_policy(SET CMP0074 NEW)

#Boost / Eigen3 / OpenCV
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)

find_package( Boost REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( CGAL REQUIRED )

include_directories(
    ${PROJECT_SOURCE_DIR}/inc 
	${PROJECT_SOURCE_DIR}/deps/autd3-library-software/client/include    
	${PROJECT_SOURCE_DIR}/deps/autd3-library-software/client/lib 
    ${Boost_INCLUDE_DIR} 
    ${EIGEN3_INCLUDE_DIR}
    )

#OpenCV
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Trajectory ../inc/Trajectory.hpp Trajectory.cpp)
set_target_properties(Trajectory PROPERTIES FOLDER "DynamicManipulation")

add_library(LinearArfModel ../inc/arfModel.hpp arfModel.cpp)
set_target_properties(LinearArfModel PROPERTIES FOLDER "DynamicManipulation")

# Tracker
# Kinect v2 tracker and Ximea stereo tracker is implemented as default trackers
option(USE_KINECT "USE KINECT" OFF)
if(USE_KINECT)
	# Find Kinect SDK v2
	find_package( KinectSDK2 REQUIRED )
	# Settings Kinect SDK v2
	include_directories( ${KinectSDK2_INCLUDE_DIRS} )
	link_directories( ${KinectSDK2_LIBRARY_DIRS} )

	add_library(KinectUtility STATIC ../inc/KinectUtility.hpp KinectUtility.cpp)
	target_link_libraries(KinectUtility ${KinectSDK2_LIBRARIES})
	set_target_properties(KinectUtility PROPERTIES FOLDER "DynamicManipulation")

	add_library(KinectSphereTracker ../inc/KinectSphereTracker.hpp KinectSphereTracker.cpp)
	target_link_libraries(KinectSphereTracker ${OpenCV_LIBS} KinectUtility)
	set_target_properties(KinectSphereTracker PROPERTIES FOLDER "DynamicManipulation")
endif()

option(USE_XIMEA "USE XIMEA CAMERA" ON)
if(USE_XIMEA)

	find_package( XIMEA REQUIRED )
	include_directories(${XIMEA_INCLUDE_DIR})

	add_library(
		CameraDevice
		../inc/CameraDevice.hpp
		CameraDevice.cpp
		../inc/xiApiPlusOcv.hpp
		xiApiPlusOcv.cpp
	)
	target_link_libraries(CameraDevice ${OpenCV_LIBS} ${XIMEA_LIBRARIES})
	set_target_properties(CameraDevice PROPERTIES FOLDER "DynamicManipulation")

	add_library(StereoTracker
		../inc/StereoTracker.hpp
		StereoTracker.cpp
		../inc/ImgProcessor.hpp
		ImgProcessor.cpp)
	target_link_libraries(StereoTracker CameraDevice ${OpenCV_LIBS})
	set_target_properties(StereoTracker PROPERTIES FOLDER "DynamicManipulation")
endif()

add_library(QPSolver
	../inc/QPSolver.h
	QPSolver.cpp
)
target_link_libraries(QPSolver CGAL::CGAL)
set_target_properties(QPSolver PROPERTIES FOLDER "DynamicManipulation")

add_library(ThrustSearch
	../inc/ThrustSearch.hpp
	ThrustSearch.cpp
)
set_target_properties(ThrustSearch PROPERTIES FOLDER "DynamicManipulation")

add_library(DynamicManipulation STATIC
	../inc/FloatingObject.hpp
	FloatingObject.cpp
	../inc/tracker.hpp
	../inc/MathUtil.hpp
	MathUtil.cpp
	../inc/manipulator.hpp
	manipulator.cpp
	../inc/geometryUtil.hpp
	geometryUtil.cpp
)
set_target_properties(DynamicManipulation PROPERTIES FOLDER "DynamicManipulation")

if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
  target_compile_definitions( DynamicManipulation PRIVATE _ENABLE_EXTENDED_ALIGNED_STORAGE )
endif()

target_link_libraries(DynamicManipulation
	autd3
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	LinearArfModel
	QPSolver
	Trajectory
)
