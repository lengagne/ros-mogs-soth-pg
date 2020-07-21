PROJECT (MogsPatternGenerator)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsPatternGenerator_VERSION_MAJOR 2)
set (MogsPatternGenerator_VERSION_MINOR 0)

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

if (NOT MogsCore_FOUND)
	find_package(MogsCore REQUIRED)
endif (NOT MogsCore_FOUND)

if( NOT MogsVisu_FOUND)
	find_package(MogsVisu QUIET)
endif( NOT MogsVisu_FOUND)

include_directories (
	include
	${MogsCore_INCLUDE_DIRS_LOCAL}
	${MogsCore_INCLUDE_DIRS}
	${MogsVisu_INCLUDE_DIRS}
	${MogsVisu_INCLUDE_DIRS_LOCAL}

)

configure_file ( "${PROJECT_SOURCE_DIR}/config_MogsPatternGenerator.h.in"
		 "${PROJECT_SOURCE_DIR}/include/config_MogsPatternGenerator.h" )

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

file(	GLOB_RECURSE
	MogsPatternGenerator_SOURCES
	src/* )

file(	GLOB_RECURSE
	MogsViewPG_SOURCES
	problem/* )


FILE ( GLOB_RECURSE
        header_files_MogsPatternGenerator
	include/*.h)

ADD_LIBRARY(	MogsPatternGenerator SHARED
		${MogsPatternGenerator_SOURCES}
)

target_link_libraries( MogsPatternGenerator
			MogsCore)

if( MogsVisu_FOUND)

ADD_LIBRARY(	MogsViewPG SHARED
		${MogsViewPG_SOURCES}
		)

target_link_libraries(  MogsViewPG
                        MogsPatternGenerator
                        ${MogsVisu_LIBRARIES})
                        
endif( MogsVisu_FOUND)

# Perform the proper linking
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsPatternGenerator)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsPatternGenerator} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsPatternGenerator LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsPatternGenerator_FOUND  TRUE CACHE INTERNAL "")
SET(MogsPatternGenerator_INCLUDE_DIRS_LOCAL  ${Include_Directories} CACHE INTERNAL "")
SET(MogsPatternGenerator_INCLUDE_DIRS  ${INSTALL_INCLUDE_DIRS} ${MogsCore_INCLUDE_DIRS} CACHE INTERNAL "")
SET(MogsPatternGenerator_LIBRARY_DIRS  "${CMAKE_INSTALL_LIBDIR}/lib" CACHE INTERNAL "")
SET(MogsPatternGenerator_LIBRARIES  MogsPatternGenerator ${MogsCore_LIBRARIES} CACHE INTERNAL "")
SET(MogsPatternGenerator_FLAGS  ${MogsCore_FLAGS} CACHE INTERNAL "")
add_definitions (${MogsPatternGenerator_FLAGS})
add_definitions(${MogsVisu_FLAGS} )
configure_file(MogsPatternGeneratorConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsPatternGeneratorConfig.cmake" @ONLY)

# install the include files
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the library files


install ( 	FILES "${PROJECT_BINARY_DIR}/MogsPatternGeneratorConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsPatternGenerator" COMPONENT dev )

if( MogsVisu_FOUND)
install( TARGETS MogsViewPG LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)
install(CODE "execute_process(COMMAND mogs2 plugins add problem mogs-pg-viewer ${INSTALL_LIB_DIRS}/libMogsViewPG.so)")
endif( MogsVisu_FOUND)

add_subdirectory(	pg_samples)
