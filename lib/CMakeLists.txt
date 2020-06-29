cmake_minimum_required(VERSION 3.8.2)

set(CMAKE_CXX_STANDARD 11)

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

# KinectSDK20
# Find Kinect SDK v2
find_package( KinectSDK2 REQUIRED )
# Settings Kinect SDK v2
include_directories( ${KinectSDK2_INCLUDE_DIRS} )
link_directories( ${KinectSDK2_LIBRARY_DIRS} )

add_library(Trajectory ../inc/Trajectory.hpp Trajectory.cpp)

add_library(KinectUtility STATIC ../inc/KinectUtility.hpp KinectUtility.cpp)
target_link_libraries(KinectUtility ${KinectSDK2_LIBRARIES})

add_library(KinectSphereTracker ../inc/KinectSphereTracker.hpp KinectSphereTracker.cpp)
target_link_libraries(KinectSphereTracker ${OpenCV_LIBS} KinectUtility)

add_library(LinearArfModel ../inc/arfModel.hpp arfModel.cpp)

add_library(GeometryUtil ../inc/geometryUtil.hpp geometryUtil.cpp)

add_library(AdditionalGain ../inc/additionalGain.hpp additionalGain.cpp)
target_link_libraries(AdditionalGain autd3)

option(USE_XIMEA "USE XIMEA CAMERA" ON)
if(USE_XIMEA)
	find_package( XIMEA REQUIRED )

	include_directories(${XIMEA_INCLUDE_DIR})

	add_library(xiApiPlusOcv ../inc/xiApiPlusOcv.hpp xiApiPlusOcv.cpp)
	target_link_libraries(xiApiPlusOcv ${OpenCV_LIBS} ${XIMEA_LIBRARIES})

	add_library(ImgProcUtil ../inc/ImgProcUtil.hpp ImgProcUtil.cpp)
	target_link_libraries(ImgProcUtil ${OpenCV_LIBS})

	add_library(CameraDevice ../inc/CameraDevice.hpp CameraDevice.cpp)
	target_link_libraries(CameraDevice xiApiPlusOcv)

	add_library(StereoTracker ../inc/StereoTracker.hpp StereoTracker.cpp)
	target_link_libraries(StereoTracker ImgProcUtil CameraDevice ${OpenCV_LIBS})
endif()

add_library(DynamicManipulation STATIC
	../inc/FloatingObject.hpp
	FloatingObject.cpp
	../inc/odcs.hpp
	odcs.cpp
	ocs.cpp
	../inc/GainPlan.hpp
	GainPlan.cpp
	Strategy.cpp
	../inc/kalmanFilter.hpp
	kalmanFilter.cpp
	../inc/EigenTools.hpp
)

if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
  target_compile_definitions( DynamicManipulation PRIVATE _ENABLE_EXTENDED_ALIGNED_STORAGE )
endif()

add_library(Projector STATIC
	../inc/projector.hpp
	projector.cpp
)

add_library(winMultiplexer STATIC
	../inc/winMultiplexer.hpp
	winMultiplexer.cpp
)

add_library(QPSolver
	../inc/QPSolver.h
	QPSolver.cpp
)
target_link_libraries(QPSolver CGAL::CGAL)
target_link_libraries(DynamicManipulation
	autd3
	AdditionalGain
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	LinearArfModel
	QPSolver
	winMultiplexer
	GeometryUtil
	Trajectory
)

add_library(dynaman_dll SHARED
	${PROJECT_SOURCE_DIR}/dll/dll.cpp
)
target_link_libraries(dynaman_dll DynamicManipulation)

