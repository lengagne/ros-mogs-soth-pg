PROJECT (MogsJointEqualityConstraint)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsJointEqualityConstraint_VERSION_MAJOR 1)
set (MogsJointEqualityConstraint_VERSION_MINOR 0)

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

if(NOT MogsSothPatternGenerator_FOUND)
find_package(MogsSothPatternGenerator REQUIRED)
endif(NOT MogsSothPatternGenerator_FOUND)

if(NOT MogsJoystick_FOUND)
find_package(MogsJoystick QUIET)
endif(NOT MogsJoystick_FOUND)

if(NOT MogsJoystick_FOUND)
    message("Mogs MogsJoystick not FOUND  for MogsJointEqualityConstraint")
endif(NOT MogsJoystick_FOUND)    


include_directories (
	include
	${MogsSothPatternGenerator_INCLUDE_DIRS}
	${MogsJoystick_INCLUDE_DIRS}
)

configure_file ("${PROJECT_SOURCE_DIR}/config_MogsJointEqualityConstraint.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_MogsJointEqualityConstraint.h" )


FILE ( GLOB_RECURSE
		header_files_MogsJointEqualityConstraint
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsJointEqualityConstraint
		src/*.cpp)

ADD_LIBRARY(	MogsJointEqualityConstraint SHARED
		${source_files_MogsJointEqualityConstraint}
)

target_link_libraries( MogsJointEqualityConstraint
			${MogsSothPatternGenerator_LIBRARIES}
			${MogsJoystick_LIBRARIES})

                
#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsJointEqualityConstraint)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsJointEqualityConstraint} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsJointEqualityConstraint LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsJointEqualityConstraint_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsJointEqualityConstraint_LIBRARIES  MogsJointEqualityConstraint CACHE INTERNAL "")
SET(MogsJointEqualityConstraint_FLAGS "-DMogsJointEqualityConstraint_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
SET(MogsJointEqualityConstraint_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsJointEqualityConstraint_FLAGS} ${MogsJoystick_FLAGS})
#  add flags (done before)
configure_file(MogsJointEqualityConstraintConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsJointEqualityConstraintConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsJointEqualityConstraintConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsJointEqualityConstraint" COMPONENT dev )


install(CODE "execute_process(COMMAND mogs2 plugins add MogsSothPatternGeneratorConstraint JointEqualityConstraint ${INSTALL_LIB_DIRS}/libMogsJointEqualityConstraint.so)")