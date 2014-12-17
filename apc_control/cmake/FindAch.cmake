# This module defines
# ACH_LIBRARY
# ACH_FOUND, if false, do not try to link 
# ACH_INCLUDE_DIR, where to find the headers
#
# $ACH_DIR is an environment variable that would
# correspond to the ./configure --prefix=$ACH_DIR
#
# Created by Eric Huang. 

FIND_PATH(ACH_INCLUDE_DIR ach.h
  PATHS
  $ENV{ACH_DIR}
  $ENV{HOME}/local
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

FIND_PATH(ACH_INCLUDE_DIR ach.h
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

FIND_PATH(ACH_INCLUDE_DIR ach.h
  PATHS
  /usr/local/include
  /usr/include
  /sw/include # Fink
  /opt/local/include # DarwinPorts
  /opt/csw/include # Blastwave
  /opt/include
  /usr/freeware/include
)

FIND_LIBRARY(ACH_LIBRARY 
  NAMES ach
  PATHS
  $ENV{ACH_DIR}
  $ENV{HOME}/local
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(ACH_LIBRARY 
  NAMES ach
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(ACH_LIBRARY 
  NAMES ach
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

SET(ACH_FOUND "NO")
IF(ACH_LIBRARY AND ACH_INCLUDE_DIR)
  SET(ACH_FOUND "YES")
ENDIF(ACH_LIBRARY AND ACH_INCLUDE_DIR)


