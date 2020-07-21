
PROJECT (MogsIKPatternGenerator)
set(PROJECT_CONTACT lengagne@gmail.com)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set (MogsIKPatternGenerator_VERSION_MAJOR 2)
set (MogsIKPatternGenerator_VERSION_MINOR 0)

find_package( MogsJoystick QUIET)

if(MogsJoystick_FOUND)
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

    include_directories ( 
            include
            ${MOGS_CORE_INCLUDE_DIRS}
    )

    configure_file ( "${PROJECT_SOURCE_DIR}/config_MogsIKPatternGenerator.h.in"
                    "${PROJECT_SOURCE_DIR}/include/config_MogsIKPatternGenerator.h" )

    link_directories(	${CMAKE_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})

    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    file(	GLOB_RECURSE
            MogsIKPatternGenerator_SOURCES
            src/* )

    FILE ( GLOB_RECURSE
            header_files_MogsIKPatternGenerator
            include/*.h)

    ADD_LIBRARY(	MogsIKPatternGenerator SHARED 
                    ${MogsIKPatternGenerator_SOURCES}
    )

    target_link_libraries( MogsIKPatternGenerator
                            MogsPatternGenerator
                            MogsJoystick)
                            
    # Perform the proper linking
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

    # install the include files
    SET(INSTALL_INCLUDE_DIRS ${CMAKE_INSTALL_INCLUDEDIR}/include/Mogs2/MogsIKPatternGenerator)
    SET(INSTALL_LIB_DIRS ${CMAKE_INSTALL_LIBDIR}/lib )

    # install the include files
    install(FILES ${header_files_MogsIKPatternGenerator} DESTINATION ${INSTALL_INCLUDE_DIRS})

    # install the library files
    install( TARGETS MogsIKPatternGenerator LIBRARY DESTINATION ${INSTALL_LIB_DIRS}  COMPONENT main)

    ### Configure file
    configure_file(MogsIKPatternGeneratorConfig.cmake.in "${PROJECT_BINARY_DIR}/MogsIKPatternGeneratorConfig.cmake" @ONLY)

    install ( 	FILES "${PROJECT_BINARY_DIR}/MogsIKPatternGeneratorConfig.cmake"
                    DESTINATION "${INSTALL_LIB_PATH}/MogsIKPatternGenerator" COMPONENT dev ) 

    install(CODE "execute_process(COMMAND mogs2 plugins add pattern_generator MogsIKPatternGenerator ${INSTALL_LIB_DIRS}/libMogsIKPatternGenerator.so)")
else(MogsJoystick_FOUND)
    message("Do not compile MogsIKPatternGenerator because MogsJoystick is not found")
endif(MogsJoystick_FOUND)
