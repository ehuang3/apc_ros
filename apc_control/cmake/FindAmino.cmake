# This module defines
# AMINO_LIBRARY
# AMINO_FOUND, if false, do not try to link 
# AMINO_INCLUDE_DIR, where to find the headers
#
# $AMINO_DIR is an environment variable that would
# correspond to the ./configure --prefix=$AMINO_DIR
#
# Created by Eric Huang. 

FIND_PATH(AMINO_INCLUDE_DIR amino.h
  PATHS
  $ENV{AMINO_DIR}
  $ENV{HOME}/local
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

FIND_PATH(AMINO_INCLUDE_DIR amino.h
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

FIND_PATH(AMINO_INCLUDE_DIR amino.h
  PATHS
  /usr/local/include
  /usr/include
  /sw/include # Fink
  /opt/local/include # DarwinPorts
  /opt/csw/include # Blastwave
  /opt/include
  /usr/freeware/include
)

FIND_LIBRARY(AMINO_LIBRARY 
  NAMES amino
  PATHS
  $ENV{AMINO_DIR}
  $ENV{HOME}/local
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(AMINO_LIBRARY 
  NAMES amino
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(AMINO_LIBRARY 
  NAMES amino
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

SET(AMINO_FOUND "NO")
IF(AMINO_LIBRARY AND AMINO_INCLUDE_DIR)
  SET(AMINO_FOUND "YES")
ENDIF(AMINO_LIBRARY AND AMINO_INCLUDE_DIR)
