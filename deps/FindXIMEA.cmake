#
##  Try to find ximea library
##
##  This module defines the following variables:
##      XIMEA_FOUND       -
##      XIMEA_INCLUDE_DIR - include directory
##      XIMEA_LIBRARIES   - full path libraries
##

SET( XIMEA_SEARCH_PATHS
    C:/XIMEA/API
    C:/XIMEA/API/x86
    C:/XIMEA/API/x64
)

find_path(XIMEA_INCLUDE_DIR
    NAMES xiApi.h
    PATHS ${XIMEA_SEARCH_PATHS}
)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    find_library(XIMEA_LIBRARY
    NAMES xiapi64.lib
    PATHS ${XIMEA_SEARCH_PATHS}
    )
else()
    find_library(XIMEA_LIBRARY
    NAMES xiapi32.lib
    PATHS ${XIMEA_SEARCH_PATHS}
    )
endif()

if( XIMEA_INCLUDE_DIR AND XIMEA_LIBRARY )
    set( XIMEA_LIBRARIES ${XIMEA_LIBRARY} )
    set( XIMEA_INCLUDE_DIRS ${XIMEA_INCLUDE_DIR} )
    set( XIMEA_FOUND TRUE )
endif()

message(STATUS "XIMEA_FOUND : ${XIMEA_FOUND}")

