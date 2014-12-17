# This module defines
# SNS_LIBRARY
# SNS_FOUND, if false, do not try to link 
# SNS_INCLUDE_DIR, where to find the headers
#
# $SNS_DIR is an environment variable that would
# correspond to the ./configure --prefix=$SNS_DIR
#
# Created by Eric Huang. 

FIND_PATH(SNS_INCLUDE_DIR sns.h
  PATHS
  $ENV{SNS_DIR}
  $ENV{HOME}/local
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

FIND_PATH(SNS_INCLUDE_DIR sns.h
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
  NO_DEFAULT_PATH
  PATH_SUFFIXES include
)

FIND_PATH(SNS_INCLUDE_DIR sns.h
  PATHS
  /usr/local/include
  /usr/include
  /sw/include # Fink
  /opt/local/include # DarwinPorts
  /opt/csw/include # Blastwave
  /opt/include
  /usr/freeware/include
)

FIND_LIBRARY(SNS_LIBRARY 
  NAMES sns
  PATHS
  $ENV{SNS_DIR}
  $ENV{HOME}/local
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(SNS_LIBRARY 
  NAMES sns
  PATHS ${CMAKE_PREFIX_PATH} # Unofficial: We are proposing this.
  NO_DEFAULT_PATH
  PATH_SUFFIXES lib64 lib
)

FIND_LIBRARY(SNS_LIBRARY 
  NAMES sns
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

SET(SNS_FOUND "NO")
IF(SNS_LIBRARY AND SNS_INCLUDE_DIR)
  SET(SNS_FOUND "YES")
ENDIF(SNS_LIBRARY AND SNS_INCLUDE_DIR)
