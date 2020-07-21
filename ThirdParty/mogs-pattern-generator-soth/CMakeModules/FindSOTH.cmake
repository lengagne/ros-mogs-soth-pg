# - Try to find SOTH
# Once done this will define
#  SOTH_FOUND - System has SOTH
#  SOTH_INCLUDE_DIRS - The SOTH include directories
#  SOTH_LIBRARY_DIRS - The library directories needed to use SOTH
#  SOTH_LIBRARIES    - The libraries needed to use SOTH


if (SOTH_INCLUDE_DIR)
  # in cache already
  SET(SOTH_FIND_QUIETLY TRUE)
endif (SOTH_INCLUDE_DIR)

find_path(SOTH_INCLUDE_DIR NAMES ActiveSet.hpp  Allocator.hpp  BaseY.hpp       Bound.hpp  debug.hpp      DestructiveColPivQR.hpp  HCOD.hpp    solvers.hpp  SubMatrix.hpp Algebra.hpp    api.hpp        BasicStage.hpp  config.hh  deprecated.hh  Givens.hpp               Random.hpp  Stage.hpp    warning.hh 
PATHS  "$ENV{SOTH_HOME}/include/soth"
	"/usr/include/soth"
	"/usr/local/include/soth"

)

find_library( SOTH_LIBRARY 
		soth
		PATHS "$ENV{SOTH_HOME}/lib"
		"/usr/lib"
		"/usr/local/lib")   

#wrong config under Debian workaround
add_definitions( -DHAVE_CSTDDEF )


set(SOTH_INCLUDE_DIRS "${SOTH_INCLUDE_DIR}" )
set(SOTH_LIBRARIES ${SOTH_LIBRARY} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBCPLEX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(SOTH  DEFAULT_MSG
					SOTH_LIBRARY SOTH_INCLUDE_DIR)

mark_as_advanced(SOTH_INCLUDE_DIR SOTH_LIBRARY )
