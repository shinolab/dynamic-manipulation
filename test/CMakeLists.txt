cmake_policy(SET CMP0074 NEW)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)

include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories( ${PROJECT_SOURCE_DIR}/deps/autd3-library-software/client/include )
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})
find_package( OpenCV REQUIRED )

add_executable(LPTest
	LPTest.cpp
)
target_link_libraries(LPTest DynamicManipulation)

add_executable(WorkSpaceTest workSpaceTest.cpp)
target_link_libraries(WorkSpaceTest DynamicManipulation)

add_executable(FindDutyTest FindDutyTest.cpp)
target_link_libraries(FindDutyTest DynamicManipulation)

add_executable(Vortex Vortex.cpp)
target_link_libraries(Vortex DynamicManipulation)

add_executable(MultiplexDutyTest MultiplexDutyTest.cpp)
target_link_libraries(MultiplexDutyTest DynamicManipulation)