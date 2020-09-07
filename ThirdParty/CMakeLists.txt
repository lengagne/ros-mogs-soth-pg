MACRO(SUBDIRLIST return_list)
    FILE(GLOB_RECURSE new_list ./mogs*/CMakeLists.txt)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

cmake_minimum_required(VERSION 3.0.2)
enable_language(CXX)
project(ros-mogs-soth-pg-thirdparty)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x -msse2")

set(CMAKE_VERBOSE_MAKEFILE OFF)
set(PROJECT_CONTACT sebastien.lengagne@univ-bpclermont.fr)

set (MoGS_VERSION_MAJOR 2)
set (MoGS_VERSION_MINOR 0)

# SET( CMAKE_BUILD_TYPE Release )

foreach(p LIB INCLUDE)
	set(var CMAKE_INSTALL_${p}DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

set(MoGS_GLOBAL_COMPILED "TRUE" CACHE INTERNAL "")

set (INSTALL_BIN_PATH bin )
set (INSTALL_LIB_PATH lib )
set (INSTALL_AR_PATH lib )
set (INSTALL_HEADERS_PATH include )
set (INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}lib)

set (LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

# add_subdirectory(soth)


add_subdirectory(mogs-utils)
add_subdirectory(mogs-3dobjects)
add_subdirectory(mogs-core)
add_subdirectory(mogs-pattern-generator)
add_subdirectory(mogs-pattern-generator-soth)
add_subdirectory(ros-mogs-constraint)
