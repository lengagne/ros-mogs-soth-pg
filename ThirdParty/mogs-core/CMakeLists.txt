cmake_minimum_required(VERSION 3.5)
project(mogs2)

set (MOGS_CORE_VERSION_MAJOR 2)
set (MOGS_CORE_VERSION_MINOR 0)

FOREACH(p LIB INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	IF(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	ENDIF()
ENDFOREACH()


file(	GLOB_RECURSE
	source_mogs_core
	src/*.cpp
)

file(	GLOB_RECURSE
	header_mogs_core
	include/*.h
	include/*.hxx
)

FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(headers SOURCES ${LibFiles})


LIST(REMOVE_ITEM source_mogs_core  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

configure_file ("${PROJECT_SOURCE_DIR}/config_mogs_core.h.in"
				"${PROJECT_SOURCE_DIR}/include/config_mogs_core.h" )

if (NOT MogsUtils_FOUND)
	find_package(MogsUtils REQUIRED)
endif (NOT MogsUtils_FOUND)

if (NOT Mogs3DObjects_FOUND)
	find_package(Mogs3DObjects REQUIRED)
endif (NOT Mogs3DObjects_FOUND)

# set the includes
include_directories(	include
			${MogsUtils_INCLUDE_DIRS}
			${MogsUtils_INCLUDE_DIRS_LOCAL}
			${Mogs3DObjects_INCLUDE_DIRS}
			${Mogs3DObjects_INCLUDE_DIRS_LOCAL}
)


link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
			${MOGS_CORE_LIBRARY_DIRS}
			${MOGSUtils_LIBRARY_DIRS}
			${Mogs3DObjects_LIBRARY_DIRS}
			)

set(MOGS_CORE_LIBRARY_DIRS "${LIBRARY_OUTPUT_PATH}" CACHE INTERNAL "" )


add_library(	MogsCore SHARED
				${source_mogs_core})

target_link_libraries( 	MogsCore
			${MogsUtils_LIBRARIES}
			${Mogs3DObjects_LIBRARIES}
			)
