include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories(${PROJECT_SOURCE_DIR}/sample)

#[[add_executable(LPTest
	LPTest.cpp
)
target_link_libraries(LPTest DynamicLevitation)

add_executable(ProfileTest
	profileTest.cpp
	../sample/profile.hpp
	../sample/profile.cpp
)

target_link_libraries(ProfileTest DynamicLevitation)

add_executable(WorkSpaceTest workSpaceTest.cpp)
target_link_libraries(WorkSpaceTest DynamicLevitation)]]

add_executable(FindDutyTest FindDutyTest.cpp)
target_link_libraries(FindDutyTest DynamicLevitation)