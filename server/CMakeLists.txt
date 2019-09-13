cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

project(sphereTrackerServer)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/deps)

include_directories(../inc)

include_directories(converter)
add_library(Server STATIC server/server.hpp server/server.cpp)
target_link_libraries(Server)
add_library(Client STATIC client/client.hpp client/client.cpp dataStruct/query.hpp)
target_link_libraries(Client)

include_directories(client)
include_directories(server)
include_directories(dataStruct)
add_executable(ServerTest 
	test/serverTest.cpp 
	dataStruct/position.hpp)
target_link_libraries(ServerTest Server DynamicManipulation)
add_executable(ClientTest
	test/clientTest.cpp
	dataStruct/position.hpp)
target_link_libraries(ClientTest Client)

