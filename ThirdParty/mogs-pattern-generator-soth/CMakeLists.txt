PROJECT (MogsSothPatternGenerator)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsSothPatternGenerator_VERSION_MAJOR 2)
set (MogsSothPatternGenerator_VERSION_MINOR 0)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(SOTH REQUIRED)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -fpic")
set(CMAKE_VERBOSE_MAKEFILE OFF)

set( LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
SET( EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin )

foreach(p LIB INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

if( NOT MogsPatternGenerator_FOUND)
	find_package(MogsPatternGenerator REQUIRED)
endif( NOT MogsPatternGenerator_FOUND)

include_directories (
    include
    ${MogsPatternGenerator_INCLUDE_DIRS_LOCAL}
    ${MogsPatternGenerator_INCLUDE_DIRS}
	${SOTH_INCLUDE_DIRS}
)

configure_file ( "${PROJECT_SOURCE_DIR}/config_MogsSothPatternGenerator.h.in"
	  "${PROJECT_SOURCE_DIR}/include/config_MogsSothPatternGenerator.h" )

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

file(	GLOB_RECURSE
    MogsSothPatternGenerator_SOURCES
    src/* 
	soth2/src/*)

FILE ( GLOB_RECURSE
	header_files_MogsSothPatternGenerator
    include/*.h
	include/*.hpp
	soth2/include/*.hpp)

ADD_LIBRARY(	MogsSothPatternGenerator SHARED
	${MogsSothPatternGenerator_SOURCES}
)

target_link_libraries( MogsSothPatternGenerator
			${MogsPatternGenerator_LIBRARIES}
			${MogsJoystick_LIBRARIES}
			soth)

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsSothPatternGenerator)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsSothPatternGenerator_INCLUDE_DIRS_LOCAL  ${Include_Directories} CACHE INTERNAL "")
SET(MogsSothPatternGenerator_INCLUDE_DIRS  ${INSTALL_INCLUDE_DIRS} 	${MogsPatternGenerator_INCLUDE_DIRS} ${SOTH_INCLUDE_DIRS} CACHE INTERNAL "")
SET(MogsSothPatternGenerator_LIBRARIES MogsSothPatternGenerator ${SOTH_LIBRARIES} ${MogsPatternGenerator_LIBRARIES}  CACHE INTERNAL "")
SET(MogsSothPatternGenerator_FLAGS "-DSOTH_FOUND -DBOOST_SIGNALS_NO_DEPRECATION_WARNING ${MogsPatternGenerator_FLAGS}" CACHE INTERNAL "")
SET(MogsSothPatternGenerator_FOUND TRUE CACHE INTERNAL "")

add_definitions(${MogsSothPatternGenerator_FLAGS})

# install the include files
install(FILES ${header_files_MogsSothPatternGenerator} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsSothPatternGenerator LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
configure_file(MogsSothPatternGeneratorConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsSothPatternGeneratorConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsSothPatternGeneratorConfig.cmake"
	DESTINATION "${INSTALL_LIB_PATH}/MogsSothPatternGenerator" COMPONENT dev )

install(CODE "execute_process(COMMAND mogs2 plugins add pattern_generator MogsSothPatternGenerator ${INSTALL_LIB_DIRS}/libMogsSothPatternGenerator.so)")

GET_DIRECTORY_PROPERTY(MogsSothPatternGenerator_INCLUDE_DIRS INCLUDE_DIRECTORIES CACHE INTERNAL "")
SET(MogsSothPatternGenerator_INCLUDE_DIRS ${MogsSothPatternGenerator_INCLUDE_DIRS} CACHE INTERNAL "")

add_subdirectory(constraints)

