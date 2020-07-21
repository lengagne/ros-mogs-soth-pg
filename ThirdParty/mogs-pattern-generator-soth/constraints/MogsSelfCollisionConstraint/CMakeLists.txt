PROJECT (MogsSelfCollisionConstraint)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsSelfCollisionConstraint_VERSION_MAJOR 1)
set (MogsSelfCollisionConstraint_VERSION_MINOR 0)

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

IF (NOT MogsBoxCollision_FOUND)
    find_package(MogsBoxCollision QUIET)
endIF (NOT MogsBoxCollision_FOUND)

include_directories (
	include
	${MogsSothPatternGenerator_INCLUDE_DIRS}
	${MogsBoxCollision_INCLUDE_DIRS}
)

configure_file ("${PROJECT_SOURCE_DIR}/config_MogsSelfCollisionConstraint.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_MogsSelfCollisionConstraint.h" )


FILE ( GLOB_RECURSE
		header_files_MogsSelfCollisionConstraint
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsSelfCollisionConstraint
		src/*.cpp)

ADD_LIBRARY(	MogsSelfCollisionConstraint SHARED
		${source_files_MogsSelfCollisionConstraint}
)

target_link_libraries( MogsSelfCollisionConstraint
			${MogsSothPatternGenerator_LIBRARIES}
			${MogsBoxCollision_LIBRARIES})

                
#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsSelfCollisionConstraint)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsSelfCollisionConstraint} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsSelfCollisionConstraint LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsSelfCollisionConstraint_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsSelfCollisionConstraint_LIBRARIES  MogsSelfCollisionConstraint CACHE INTERNAL "")
SET(MogsSelfCollisionConstraint_FLAGS "-DMogsSelfCollisionConstraint_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
SET(MogsSelfCollisionConstraint_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsSelfCollisionConstraint_FLAGS})
#  add flags (done before)
configure_file(MogsSelfCollisionConstraintConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsSelfCollisionConstraintConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsSelfCollisionConstraintConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsSelfCollisionConstraint" COMPONENT dev )


install(CODE "execute_process(COMMAND mogs2 plugins add MogsSothPatternGeneratorConstraint SelfCollisionConstraint ${INSTALL_LIB_DIRS}/libMogsSelfCollisionConstraint.so)")