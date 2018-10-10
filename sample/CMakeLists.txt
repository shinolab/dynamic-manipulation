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

add_executable(interaction interaction.cpp)
include_directories(../inc)
target_link_libraries(interaction DynamicLevitation)

add_executable(projection projection.cpp)
include_directories(../inc)
target_link_libraries(projection DynamicLevitation Projector)

add_executable(sequential sequential.cpp)
include_directories(../inc)
target_link_libraries(sequential DynamicLevitation)