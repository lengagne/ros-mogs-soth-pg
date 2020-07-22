PROJECT (MogsSothPatternGenerator)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsSothPatternGenerator_VERSION_MAJOR 2)
set (MogsSothPatternGenerator_VERSION_MINOR 0)

SET(CMAKE_MODULE_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

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

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs)

find_package(Qt4 REQUIRED COMPONENTS QTCORE QTXML QtXmlPatterns)
#find_package(Qt4 QUIET COMPONENTS QTGUI)

set(QT_DEFINITIONS -DQT_NO_KEYWORDS)
if(QT_QTGUI_FOUND)
set(QT_DEFINITIONS ${QT_DEFINITIONS} -DQT_GUI_FOUND)
endif(QT_QTGUI_FOUND)
add_definitions(${QT_DEFINITIONS})

include(${QT_USE_FILE})


include_directories (
	include
	${catkin_INCLUDE_DIRS}
	${QT_INCLUDES}
)

configure_file ("${PROJECT_SOURCE_DIR}/config_ROSMogsConstraint.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_ROSMogsConstraint.h" )


FILE ( GLOB_RECURSE
		header_files_ROSMogsConstraint
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_ROSMogsConstraint
		src/*.cpp)

ADD_LIBRARY(	ROSMogsConstraint SHARED
		${source_files_ROSMogsConstraint}
)

target_link_libraries( ROSMogsConstraint
                        ${catkin_LIBRARIES}
                        ${QT_LIBRARIES}
			)
#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/ROSMogsConstraint)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_ROSMogsConstraint} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS ROSMogsConstraint LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(ROSMogsConstraint_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(ROSMogsConstraint_LIBRARIES  ROSMogsConstraint ${MogsEndEffectorConstraint_FLAGS} CACHE INTERNAL "")
SET(ROSMogsConstraint_FLAGS "-DROSMogsConstraint_FOUND " CACHE INTERNAL "")
SET(ROSMogsConstraint_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${ROSMogsConstraint_FLAGS})
#  add flags (done before)
configure_file(ROSMogsConstraintConfig.cmake.in "${PROJECT_BINARY_DIR}/ROSMogsConstraintConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/ROSMogsConstraintConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/ROSMogsConstraint" COMPONENT dev )


add_subdirectory(constraints)

