cmake_minimum_required(VERSION 3.8.2)

enable_language(CSharp)

set (CMAKE_CXX_STANDARD 11)
cmake_policy(SET CMP0074 NEW)

project(DynamicManipulation)

#add_subdirectory(deps/autd3/client)
add_subdirectory(deps/autd3-library-software/client/deps/BeckhoffADS)
add_subdirectory(deps/autd3-library-software/client/lib)

#Boost / Eigen3 / OpenCV
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)

find_package( Boost REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( CGAL REQUIRED )

include_directories(inc/ deps/autd3-library-software/client/include ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

#OpenCV
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

# KinectSDK20
# Find Kinect SDK v2
find_package( KinectSDK2 REQUIRED )

# Settings Kinect SDK v2
include_directories( ${KinectSDK2_INCLUDE_DIRS} )
link_directories( ${KinectSDK2_LIBRARY_DIRS} )

#Dlib
include_directories(deps/dlib-19.0)

add_library(KinectApp STATIC inc/KinectApp.hpp src/KinectApp.cpp)
target_link_libraries(KinectApp ${KinectSDK2_LIBRARIES})

add_library(DynamicManipulation STATIC
 inc/odcs.hpp
 src/odcs.cpp
 inc/additionalGain.hpp
 src/additionalGain.cpp
 src/ocs.cpp
 src/ods.cpp
 inc/KinectPositionSensor.hpp
 src/KinectPositionSensor.cpp
 src/FloatingObject.cpp
 inc/read-csv-to-eigen.hpp
 src/read-csv-to-eigen.cpp
 inc/arfModel.hpp
 src/arfModel.cpp
 inc/kalmanFilter.hpp
 src/kalmanFilter.cpp
 src/Trajectory.cpp
 inc/EigenTools.hpp
 inc/geometryUtil.hpp
 src/geometryUtil.cpp
)

source_group("sensor" FILES 
	src/KinectPositionSensor.cpp
)

if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
  target_compile_definitions( DynamicManipulation PRIVATE _ENABLE_EXTENDED_ALIGNED_STORAGE )
endif()

add_library(Projector STATIC
	inc/projector.hpp
	src/projector.cpp
)

add_library(winMultiplexer STATIC
	inc/winMultiplexer.hpp
	src/winMultiplexer.cpp
)

add_library(QPSolver
	inc/QPSolver.h
	src/QPSolver.cpp
)
target_link_libraries(QPSolver CGAL::CGAL)
target_link_libraries(DynamicManipulation autd3
 ${Boost_LIBRARIES}
 ${OpenCV_LIBS}
 KinectApp
 QPSolver
 winMultiplexer
)

add_library(dynaman_dll SHARED
	dll/dll.cpp
)
target_link_libraries(dynaman_dll DynamicManipulation)

add_executable(simple_cs
	cs_api/simple.cs
	cs_api/dynaman.cs
)
option(USE_MATHNET "use Math.Net" ON)
if(USE_MATHNET)
	add_definitions(-DENABLE_MATHNET)
	target_link_libraries(simple_cs dynaman_dll cs_api/deps/MathNet.Numerics)
else()
	target_link_libraries(simple_cs dynaman_dll)
endif()

target_compile_options(simple_cs PUBLIC "/unsafe")

option(BUILD_SAMPLES "build sample programs" ON)
if(BUILD_SAMPLES)
	add_subdirectory(sample)
endif()

option(BUILD_TEST "build test programs" ON)
if(BUILD_TEST)
	add_subdirectory(test)
endif()
add_subdirectory(server)
