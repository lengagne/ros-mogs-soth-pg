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

if (NOT MogsJoystick_FOUND)
	find_package(MogsJoystick QUIET)
endif (NOT MogsJoystick_FOUND)

# set the includes
include_directories(	include
			${MogsUtils_INCLUDE_DIRS}
			${MogsUtils_INCLUDE_DIRS_LOCAL}
			${Mogs3DObjects_INCLUDE_DIRS}
			${Mogs3DObjects_INCLUDE_DIRS_LOCAL}
			${MogsJoystick_INCLUDE_DIRS}
			${MogsJoystick_INCLUDE_DIRS_LOCAL})


link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
			${MOGS_CORE_LIBRARY_DIRS}
			${MOGSUtils_LIBRARY_DIRS}
			${Mogs3DObjects_LIBRARY_DIRS}
			${MogsJoystick_LIBRARY_DIRS}
			)

set(MOGS_CORE_LIBRARY_DIRS "${LIBRARY_OUTPUT_PATH}" CACHE INTERNAL "" )


add_library(	MogsCore SHARED
				${source_mogs_core})

target_link_libraries( 	MogsCore
			${MogsUtils_LIBRARIES}
			${Mogs3DObjects_LIBRARIES}
			${MogsJoystick_LIBRARIES})

add_executable(mogs2 src/main.cpp)

target_link_libraries(	mogs2
			MogsCore)

### Completion for non graphical interface
add_executable(	MogsCompletion
		"${CMAKE_CURRENT_SOURCE_DIR}/tools/MogsCompletion.cpp")

target_link_libraries(	MogsCompletion
			MogsCore
			)

### Configure file

### installation of the files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsCore CACHE INTERVAL "")
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsCore_INCLUDE_DIRS_LOCAL  ${Include_Directories} CACHE INTERNAL "")
SET(MogsCore_INCLUDE_DIRS  ${INSTALL_INCLUDE_DIRS} ${MogsUtils_INCLUDE_DIRS} ${Mogs3DObjects_INCLUDE_DIRS} ${MogsJoystick_INCLUDE_DIRS} CACHE INTERNAL "")
SET(MogsCore_LIBRARY_DIRS  "${CMAKE_INSTALL_LIBDIR}/lib" CACHE INTERNAL "")
SET(MogsCore_LIBRARIES MogsCore ${MogsUtils_LIBRARIES} ${Mogs3DObjects_LIBRARIES} CACHE INTERNAL "")
SET(MogsCore_FLAGS ${MogsUtils_FLAGS} ${Mogs3DObjects_FLAGS} ${MogsJoystick_FLAGS} CACHE INTERNAL "")
SET(MogsCore_FOUND "TRUE" CACHE INTERNAL "")
configure_file(MogsCoreConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsCoreConfig.cmake" @ONLY)

add_definitions(${MogsCore_FLAGS})

install( FILES ${header_mogs_core} DESTINATION ${INSTALL_INCLUDE_DIRS})
install( TARGETS MogsCompletion mogs2  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install( TARGETS MogsCore LIBRARY DESTINATION ${INSTALL_LIB_DIRS})

if(NOT CRRI)
install( FILES "tools/MogsCompletion.sh" DESTINATION /etc/bash_completion.d/)
endif(NOT CRRI)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsCoreConfig.cmake"
		DESTINATION "${INSTALL_LIB_DIRS}/MogsCore" COMPONENT dev )


message ("install lib path = " ${INSTALL_LIB_PATH})

install(DIRECTORY 	${CMAKE_CURRENT_SOURCE_DIR}/data/templates
                    ${CMAKE_CURRENT_SOURCE_DIR}/data/xsd
	DESTINATION 	${CMAKE_INSTALL_LIBDIR}/share/mogs2)

install(CODE "execute_process(COMMAND ldconfig)")

# add_subdirectory(plugins)
