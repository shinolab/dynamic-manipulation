cmake_minimum_required(VERSION 3.8.2)

set (CMAKE_CXX_STANDARD 17)

if(POLICY CMP0074)
	cmake_policy(SET CMP0074 NEW)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS TRUE)

project(DynamicManipulation)

#add_subdirectory(deps/autd3/client)
add_subdirectory(deps/autd3-library-software/client/deps/BeckhoffADS)
add_subdirectory(deps/autd3-library-software/client/lib)
set_target_properties(autd3 PROPERTIES FOLDER "3rdParty")
set_target_properties(ads PROPERTIES FOLDER "3rdParty")
add_subdirectory(lib)

option(BUILD_SAMPLES "build sample programs" ON)
if(BUILD_SAMPLES)
	add_subdirectory(sample)
endif()
