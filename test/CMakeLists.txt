include_directories(${PROJECT_SOURCE_DIR}/inc)

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