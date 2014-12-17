# Find TCLAP
#
# This sets the following variables:
# TCLAP_FOUND
# TCLAP_INCLUDE_DIRS

find_path(TCLAP_INCLUDE_DIR
    NAMES tclap/CmdLine.h
    PATHS "${CMAKE_INSTALL_PREFIX}/include"
    PATH_SUFFIXES tclap)

set(TCLAP_INCLUDE_DIR ${TCLAP_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TCLAP DEFAULT_MSG TCLAP_INCLUDE_DIR)

mark_as_advanced(TCLAP_INCLUDE_DIR)
