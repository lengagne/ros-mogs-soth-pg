PROJECT (Mogs3DObjects)
set(PROJECT_CONTACT lengagne@gmail.com)

set(MOGS3DOBJECT_VERSION_MAJOR 1)	# use of QT
set(MOGS3DOBJECT_VERSION_MINOR 0)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
set(CMAKE_VERBOSE_MAKEfile OFF)

set(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

FOREACH(p LIB INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	IF(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	ENDIF()
ENDFOREACH()

FIND_PACKAGE(Boost 1.40.0 QUIET COMPONENTS serialization)
FIND_PACKAGE( Qt4 REQUIRED COMPONENTS QtCore )
FIND_PACKAGE( Qt4 COMPONENTS QtGui QtOpenGL)

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

IF(NOT MoGS_GLOBAL_COMPILED)
	add_custom_target(test_mogs)
ENDIF(NOT MoGS_GLOBAL_COMPILED)

if (NOT MogsUtils_FOUND)
	find_package(MogsUtils REQUIRED)
	message("MogsUtils_INCLUDE_DIRS = " ${MogsUtils_INCLUDE_DIRS})
endif (NOT MogsUtils_FOUND)

INCLUDE_DIRECTORIES(	include
			${MogsUtils_INCLUDE_DIRS}
			${MogsUtils_INCLUDE_DIRS_LOCAL}
			${Boost_INCLUDE_DIRS}
                        ${OPENGL_INCLUDE_DIRS}
                        ${GLUT_INCLUDE_DIRS}
			)

link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
                    ${MogsUtils_LIBRARY_DIRS})

file(	GLOB_RECURSE
	3DOBJECT_SOURCES
	src/* )

file ( GLOB_RECURSE
	header_files_Mogs3DObjects
	include/*.h	)

add_library(	Mogs3DObjects SHARED
		${3DOBJECT_SOURCES})

target_link_libraries(	Mogs3DObjects
			${Boost_LIBRARIES}
			${QT_LIBRARIES}
            ${OPENGL_LIBRARIES}
            ${GLUT_LIBRARIES}
			${MogsUtils_LIBRARIES}
			)


add_executable(	Mogs3DObjects_test	EXCLUDE_FROM_ALL
		${header_files_Mogs3DObjects}
		"${CMAKE_CURRENT_SOURCE_DIR}/test/main.cpp")

target_link_libraries(	Mogs3DObjects_test
			Mogs3DObjects)

add_executable(	Mogs3DObjects_test_serialization	EXCLUDE_FROM_ALL
		${header_files_MoGS_RBDL}
		"${CMAKE_CURRENT_SOURCE_DIR}/test/serialization.cpp")

target_link_libraries(	Mogs3DObjects_test_serialization
                        Mogs3DObjects)

add_executable( mogs_object_affine
                apps/Object_affine.cpp)

target_link_libraries(	mogs_object_affine
                        Mogs3DObjects)

add_executable( mogs_object_add_point
                apps/Object_add_point.cpp)

target_link_libraries(	mogs_object_add_point
                        Mogs3DObjects)


add_custom_target(	test_mogs3dobject
			COMMAND make Mogs3DObjects_test
			COMMAND make Mogs3DObjects_test_serialization)

#add_dependencies(	test_mogs test_mogs3dobject)

set(Mogs3DObjects_FLAGS  ${Mogs3DObjects_FLAGS} "-DMogs3dObject_FOUND " CACHE INTERNAL "")
if(Boost_SERIALIZATION_FOUND)
	set(Mogs3DObjects_FLAGS  "-DBoost_SERIALIZATION_FOUND " ${Mogs3DObjects_FLAGS} CACHE INTERNAL "")
endif(Boost_SERIALIZATION_FOUND)
add_definitions(${Mogs3DObjects_FLAGS})

# Perform the proper linking
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

configure_file ( "${PROJECT_SOURCE_DIR}/config_3DOBJECT.h.in"
		 "${PROJECT_SOURCE_DIR}/include/config_3DOBJECT.h" )

# install the include files
set(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/Mogs3DObjects)
set(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_Mogs3DObjects} DESTINATION ${INSTALL_INCLUDE_DIRS})
# install the library files
install( TARGETS "Mogs3DObjects" LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(Mogs3DObjects_INCLUDE_DIRS_LOCAL  ${Include_Directories} CACHE INTERNAL "")
SET(Mogs3DObjects_INCLUDE_DIRS  ${INSTALL_INCLUDE_DIRS} 	${MogsUtils_INCLUDE_DIRS}	${Boost_INCLUDE_DIRS}	${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS}  CACHE INTERNAL "")
SET(Mogs3DObjects_LIBRARY_DIRS  "${CMAKE_INSTALL_LIBDIR}/lib" CACHE INTERNAL "")
SET(Mogs3DObjects_LIBRARIES  Mogs3DObjects 			${Boost_LIBRARIES}   ${OPENGL_LIBRARIES}   ${GLUT_LIBRARIES} ${MogsUtils_LIBRARIES} CACHE INTERNAL "")
SET(Mogs3DObjects_FOUND "TRUE" CACHE INTERNAL "")


configure_file(Mogs3DObjectsConfig.cmake.in "${PROJECT_BINARY_DIR}/Mogs3DObjectsConfig.cmake" @ONLY)

set ( INSTALL_LIB_PATH lib )

install ( 	FILES "${PROJECT_BINARY_DIR}/Mogs3DObjectsConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/Mogs3DObjects" COMPONENT dev )

