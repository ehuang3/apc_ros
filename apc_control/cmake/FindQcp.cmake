# This module defines
# QCP_LIBRARY
# QCP_FOUND, if false, do not try to link 
# QCP_INCLUDE_DIR, where to find the headers
#
# $QCP_DIR is an environment variable that would
# correspond to the ./configure --prefix=$QCP_DIR
#
# Created by Eric Huang. 

FIND_PATH(QCP_INCLUDE_DIR qcustomplot.h
  PATHS
  $ENV{QCP_DIR}
  $ENV{HOME}/local
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

FIND_PATH(QCP_INCLUDE_DIR qcustomplot.h
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

FIND_PATH(QCP_INCLUDE_DIR qcustomplot.h
  PATHS
  /usr/local/include
  /usr/include
  /sw/include # Fink
  /opt/local/include # DarwinPorts
  /opt/csw/include # Blastwave
  /opt/include
  /usr/freeware/include
)

FIND_LIBRARY(QCP_LIBRARY 
  NAMES qcustomplot
  PATHS
  $ENV{QCP_DIR}
  $ENV{HOME}/local
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(QCP_LIBRARY 
  NAMES qcustomplot
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(QCP_LIBRARY 
  NAMES qcustomplot
  PATHS
  /usr/local
  /usr
  /sw
  /opt/local
  /opt/csw
  /opt
  /usr/freeware
  PATH_SUFFIXES lib64 lib
)

SET(QCP_FOUND "NO")
IF(QCP_LIBRARY AND QCP_INCLUDE_DIR)
  SET(QCP_FOUND "YES")
  SET_PROPERTY(
    GLOBAL
    APPEND
    PROPERTY COMPILE_DEFINITIONS "QCUSTOMPLOT_USE_LIBRARY"
    )
ENDIF(QCP_LIBRARY AND QCP_INCLUDE_DIR)


