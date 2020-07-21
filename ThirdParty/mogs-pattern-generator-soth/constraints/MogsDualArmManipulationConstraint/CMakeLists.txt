PROJECT (MogsDualArmManipulationConstraint)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsDualArmManipulationConstraint_VERSION_MAJOR 1)
set (MogsDualArmManipulationConstraint_VERSION_MINOR 0)

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

include_directories (
	include
	${MogsSothPatternGenerator_INCLUDE_DIRS}
)

configure_file ("${PROJECT_SOURCE_DIR}/config_MogsDualArmManipulationConstraint.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_MogsDualArmManipulationConstraint.h" )


FILE ( GLOB_RECURSE
		header_files_MogsDualArmManipulationConstraint
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_MogsDualArmManipulationConstraint
		src/*.cpp)

ADD_LIBRARY(	MogsDualArmManipulationConstraint SHARED
		${source_files_MogsDualArmManipulationConstraint}
)

target_link_libraries( MogsDualArmManipulationConstraint
			${MogsSothPatternGenerator_LIBRARIES})


#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsDualArmManipulationConstraint)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_MogsDualArmManipulationConstraint} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS MogsDualArmManipulationConstraint LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(MogsDualArmManipulationConstraint_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(MogsDualArmManipulationConstraint_LIBRARIES  MogsDualArmManipulationConstraint CACHE INTERNAL "")
SET(MogsDualArmManipulationConstraint_FLAGS "-DMogsDualArmManipulationConstraint_FOUND ${IPOPT_FLAGS}" CACHE INTERNAL "")
SET(MogsDualArmManipulationConstraint_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${MogsDualArmManipulationConstraint_FLAGS})
#  add flags (done before)
configure_file(MogsDualArmManipulationConstraintConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsDualArmManipulationConstraintConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/MogsDualArmManipulationConstraintConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/MogsDualArmManipulationConstraint" COMPONENT dev )


install(CODE "execute_process(COMMAND mogs2 plugins add MogsSothPatternGeneratorConstraint DualArmManipulationConstraint ${INSTALL_LIB_DIRS}/libMogsDualArmManipulationConstraint.so)")
