cmake_minimum_required(VERSION 3.5)
project(mogs2_utils)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

FOREACH(p LIB INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	IF(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	ENDIF()
ENDFOREACH()

### QT used for graphical interfaces strings and xml
find_package(Qt4 REQUIRED COMPONENTS QTCORE QTXML QtXmlPatterns QUIET COMPONENTS QTGUI)
#find_package(Qt4 QUIET COMPONENTS QTGUI)

set(QT_DEFINITIONS -DQT_NO_KEYWORDS)
if(QT_QTGUI_FOUND)
set(QT_DEFINITIONS ${QT_DEFINITIONS} -DQT_GUI_FOUND)
endif(QT_QTGUI_FOUND)
add_definitions(${QT_DEFINITIONS})

include(${QT_USE_FILE})


### Eigen is used for matrix computations
find_package(PkgConfig REQUIRED)
if(CRRI)
	message("We are on the CCRI")
	set(EIGEN3_INCLUDE_DIR "/opt/apps/gcc-4.8.4/eigen-3.3.1/include/eigen3")
else(CRRI)
	find_package(Eigen3 REQUIRED)
endif(CRRI)

## TCMALLOC used to optimized memory allocation
find_package(PkgConfig REQUIRED)
pkg_check_modules(TCMALLOC libtcmalloc_minimal)
if(TCMALLOC_FOUND)
  include_directories(${TCMALLOC_INCLUDE_DIRS})
  link_directories(${TCMALLOC_LIBRARY_DIRS})
  list(APPEND MG_LIBRARIES ${TCMALLOC_LIBRARIES})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free")
endif(TCMALLOC_FOUND)

file(	GLOB_RECURSE
	source_mogs_utils
	src/*.cpp
)

file(	GLOB_RECURSE
	header_mogs_core
	include/*.h
	include/*.hxx
)

configure_file ( "${PROJECT_SOURCE_DIR}/config_mogs_utils.h.in"
		 "${PROJECT_SOURCE_DIR}/include/config_mogs_utils.h" )

include_directories(	include
                        ${TCMALLOC_INCLUDE_DIRS}
                        ${EIGEN3_INCLUDE_DIR}
                        ${QT_INCLUDES})


link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}
                    ${TCMALLOC_LIBRARY_DIRS})

add_library(	MogsUtils SHARED
				${source_mogs_utils} )

target_link_libraries( MogsUtils
						${QT_LIBRARIES}
						${TCMALLOC_LIBRARIES}
)

set (INSTALL_LIB_PATH lib )

SET(MogsUtils_INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsUtils CACHE INTERNAL "")
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )
install( FILES ${header_mogs_core} DESTINATION ${MogsUtils_INSTALL_INCLUDE_DIRS})
install( TARGETS MogsUtils LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsUtils_INCLUDE_DIRS_LOCAL ${Include_Directories}  CACHE INTERNAL "")
SET(MogsUtils_INCLUDE_DIRS  ${MogsUtils_INSTALL_INCLUDE_DIRS}   ${EIGEN3_INCLUDE_DIR} ${TCMALLOC_INCLUDE_DIRS} ${QT_INCLUDES} CACHE INTERNAL "")

SET(MogsUtils_LIBRARY_DIRS  "${CMAKE_INSTALL_LIBDIR}/lib ${TCMALLOC_LIBRARY_DIRS}"  CACHE INTERNAL "")
SET(MogsUtils_LIBRARIES  MogsUtils ${QT_LIBRARIES} CACHE INTERNAL "")
SET(MogsUtils_FLAGS "-DMogsUtils_FOUND ${QT_DEFINITIONS} -std=c++0x -msse2 -DQT_NO_KEYWORDS  -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free" CACHE INTERNAL "")
SET(MogsUtils_FOUND "TRUE" CACHE INTERNAL "")
configure_file(MogsUtilsConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsUtilsConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsUtilsConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsUtils" COMPONENT dev )

