cmake_minimum_required(VERSION 3.8.2)

enable_language(CSharp)

set (CMAKE_CXX_STANDARD 11)

project(DynamicLevitation)

#add_subdirectory(deps/autd3/client)
add_subdirectory(deps/autd3-software/client/deps/BeckhoffADS)
add_subdirectory(deps/autd3-software/client/lib)

#Boost / Eigen3 / OpenCV
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)

find_package( Boost REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( CGAL REQUIRED )

include_directories(inc/ deps/autd3-software/client/include ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

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

add_library(DynamicLevitation STATIC
 inc/odcs.hpp
 src/odcs.cpp
 inc/additionalGain.hpp
 src/additionalGain.cpp
 inc/KinectApp.hpp
 src/KinectApp.cpp
 src/ocs.cpp
 src/ods.cpp
 src/FloatingObject.cpp
 inc/read-csv-to-eigen.hpp
 src/read-csv-to-eigen.cpp
 inc/arfModel.hpp
 src/arfModel.cpp
 inc/kalmanFilter.hpp
 src/kalmanFilter.cpp
 inc/QPSolver.h
 src/Trajectory.cpp
 inc/EigenTools.hpp
)

if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
  target_compile_definitions( DynamicLevitation PRIVATE _ENABLE_EXTENDED_ALIGNED_STORAGE )
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
target_link_libraries(DynamicLevitation autd3
 ${Boost_LIBRARIES}
 ${OpenCV_LIBS}
 ${KinectSDK2_LIBRARIES}
 QPSolver
 winMultiplexer
)

add_library(dynaman_dll SHARED
	dll/dll.cpp
)
target_link_libraries(dynaman_dll DynamicLevitation)

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

add_subdirectory(sample)
add_subdirectory(test)
