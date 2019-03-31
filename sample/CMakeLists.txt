cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

# Eigen3
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

add_executable(simple simple.cpp)
include_directories(../inc)
target_link_libraries(simple DynamicLevitation)

add_executable(pipeline pipeline.cpp)
include_directories(../inc)
target_link_libraries(pipeline DynamicLevitation)

add_executable(projection projection.cpp)
include_directories(../inc)
target_link_libraries(projection DynamicLevitation Projector)

add_executable(sequential sequential.cpp)
include_directories(../inc)
target_link_libraries(sequential DynamicLevitation)

add_executable(impactor impactor.cpp
	../inc/rootFindingPoly.hpp)
target_link_libraries(impactor DynamicLevitation)

add_executable(single_sided single_sided.cpp)
target_link_libraries(single_sided DynamicLevitation)

add_executable(ss_trajectory single_sided_with_trajectory.cpp)
target_link_libraries(ss_trajectory DynamicLevitation)

add_executable(demo_proactive demo_proactive.cpp)
target_link_libraries(demo_proactive DynamicLevitation)

add_executable(demo_guidance demo_guidance.cpp init_uist.hpp)
target_link_libraries(demo_guidance DynamicLevitation Projector)

add_executable(demo_notification demo_notification.cpp init_uist.hpp)
target_link_libraries(demo_notification DynamicLevitation Projector)

add_executable(demo_avatar demo_avatar.cpp init_uist.hpp)
target_link_libraries(demo_avatar DynamicLevitation Projector)

add_executable(demo_volleyball demo_volleyball.cpp init_uist.hpp)
target_link_libraries(demo_volleyball DynamicLevitation Projector)