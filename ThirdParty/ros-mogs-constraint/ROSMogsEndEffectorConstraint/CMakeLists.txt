PROJECT (ROSMogsEndEffectorConstraint)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (ROSMogsEndEffectorConstraint_VERSION_MAJOR 1)
set (ROSMogsEndEffectorConstraint_VERSION_MINOR 0)

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

if(NOT MogsEndEffectorConstraint_FOUND)
find_package(MogsEndEffectorConstraint REQUIRED)
endif(NOT MogsEndEffectorConstraint_FOUND)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs)


# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros-mogs-soth-pg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
# )


include_directories (
	include
	${MogsEndEffectorConstraint_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

configure_file ("${PROJECT_SOURCE_DIR}/config_ROSMogsEndEffectorConstraint.h.in"
                "${PROJECT_SOURCE_DIR}/include/config_ROSMogsEndEffectorConstraint.h" )


FILE ( GLOB_RECURSE
		header_files_ROSMogsEndEffectorConstraint
		include/*.h)

FILE ( GLOB_RECURSE
		source_files_ROSMogsEndEffectorConstraint
		src/*.cpp)

ADD_LIBRARY(	ROSMogsEndEffectorConstraint SHARED
		${source_files_ROSMogsEndEffectorConstraint}
)

target_link_libraries( ROSMogsEndEffectorConstraint
			${MogsEndEffectorConstraint_LIBRARIES}
                        ${catkin_LIBRARIES}
			)

                
#
# install the include files
SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/ROSMogsEndEffectorConstraint)
SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

# install the include files
install(FILES ${header_files_ROSMogsEndEffectorConstraint} DESTINATION ${INSTALL_INCLUDE_DIRS})

# install the library files
install( TARGETS ROSMogsEndEffectorConstraint LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

### Configure file
get_property(Include_Directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
SET(ROSMogsEndEffectorConstraint_INCLUDE_DIRS  ${Include_Directories} CACHE INTERNAL "")
SET(ROSMogsEndEffectorConstraint_LIBRARIES  ROSMogsEndEffectorConstraint ${MogsEndEffectorConstraint_FLAGS} CACHE INTERNAL "")
SET(ROSMogsEndEffectorConstraint_FLAGS "-DROSMogsEndEffectorConstraint_FOUND " CACHE INTERNAL "")
SET(ROSMogsEndEffectorConstraint_FOUND "TRUE" CACHE INTERNAL "")
add_definitions(${ROSMogsEndEffectorConstraint_FLAGS})
#  add flags (done before)
configure_file(ROSMogsEndEffectorConstraintConfig.cmake.in "${PROJECT_BINARY_DIR}/ROSMogsEndEffectorConstraintConfig.cmake" @ONLY)

install ( 	FILES "${PROJECT_BINARY_DIR}/ROSMogsEndEffectorConstraintConfig.cmake"
		DESTINATION "${INSTALL_LIB_PATH}/ROSMogsEndEffectorConstraint" COMPONENT dev )


install(CODE "execute_process(COMMAND mogs2 plugins add MogsSothPatternGeneratorConstraint ROSEndEffectorConstraint ${INSTALL_LIB_DIRS}/libROSMogsEndEffectorConstraint.so)")
