cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

# Eigen3
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

add_executable(simple simple.cpp)
include_directories(../inc)
target_link_libraries(simple DynamicManipulation)

add_executable(pipeline pipeline.cpp)
include_directories(../inc)
target_link_libraries(pipeline DynamicManipulation)

add_executable(projection projection.cpp)
include_directories(../inc)
target_link_libraries(projection DynamicManipulation Projector)

add_executable(sequential sequential.cpp)
include_directories(../inc)
target_link_libraries(sequential DynamicManipulation)

add_executable(impactor impactor.cpp
	../inc/rootFindingPoly.hpp)
target_link_libraries(impactor DynamicManipulation)

add_executable(single_sided single_sided.cpp)
target_link_libraries(single_sided DynamicManipulation)

#add_executable(ss_trajectory single_sided_with_trajectory.cpp)
#target_link_libraries(ss_trajectory DynamicManipulation)

add_executable(demo_proactive demo_proactive.cpp)
target_link_libraries(demo_proactive DynamicManipulation)

#add_executable(demo_guidance demo_guidance.cpp init_uist.hpp)
#target_link_libraries(demo_guidance DynamicManipulation Projector)

#add_executable(demo_notification demo_notification.cpp init_uist.hpp)
#target_link_libraries(demo_notification DynamicManipulation Projector)

add_executable(demo_avatar demo_avatar.cpp init_uist.hpp)
target_link_libraries(demo_avatar DynamicManipulation Projector)

#add_executable(demo_instruction demo_instruction.cpp init_uist.hpp)
#target_link_libraries(demo_instruction DynamicManipulation Projector)

#add_executable(demo_volleyball demo_volleyball.cpp init_uist.hpp)
#target_link_libraries(demo_volleyball DynamicManipulation Projector)

add_executable(demo_innovative demo_innovative.cpp init_uist.hpp)
target_link_libraries(demo_innovative DynamicManipulation)