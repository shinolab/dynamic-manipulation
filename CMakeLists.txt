cmake_minimum_required(VERSION 3.8.2)

enable_language(CSharp)

set (CMAKE_CXX_STANDARD 17)
cmake_policy(SET CMP0074 NEW)

set_property(GLOBAL PROPERTY USE_FOLDERS TRUE)

project(DynamicManipulation)

#add_subdirectory(deps/autd3/client)
add_subdirectory(deps/autd3-library-software/client/deps/BeckhoffADS)
add_subdirectory(deps/autd3-library-software/client/lib)
set_target_properties(autd3 PROPERTIES FOLDER "3rdParty")
set_target_properties(ads PROPERTIES FOLDER "3rdParty")
add_subdirectory(lib)

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

option(BUILD_SAMPLES "build sample programs" ON)
if(BUILD_SAMPLES)
	add_subdirectory(sample)
endif()

option(BUILD_SERVER "build dynaman server" ON)
if(BUILD_SERVER)
	add_subdirectory(server)
endif()