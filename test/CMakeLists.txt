cmake_minimum_required( VERSION 3.12)

SET(CMAKE_CXX_STANDARD 17)

cmake_policy(SET CMP0074 NEW)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)

include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories( ${PROJECT_SOURCE_DIR}/deps/autd3-library-software/client/include )
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})
find_package( OpenCV REQUIRED )

# Find Kinect SDK v2
find_package( KinectSDK2 REQUIRED )
include_directories( ${KinectSDK2_INCLUDE_DIRS} )
link_directories( ${KinectSDK2_LIBRARY_DIRS} )


add_executable(WorkSpaceTest workSpaceTest.cpp)
target_link_libraries(WorkSpaceTest DynamicManipulation)

add_executable(FindDutyTest FindDutyTest.cpp)
target_link_libraries(FindDutyTest DynamicManipulation)

add_executable(MultiplexDutyTest MultiplexDutyTest.cpp)
target_link_libraries(MultiplexDutyTest DynamicManipulation)

# add_executable(TrajectoryTest TrajectoryTest.cpp)
# target_link_libraries(TrajectoryTest Trajectory)

add_executable(ArfModelTest ArfModelTest.cpp)
target_link_libraries(ArfModelTest LinearArfModel)

add_executable(MultiplexerTest multiplexerTest.cpp)
target_link_libraries(MultiplexerTest WinMultiplexer)
