#--------------------------------------------------------
# The CMakeLists.txt for:                  lib_contactmgr
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

SET(SRC
  CMAlert.cpp
  ContactMgrCore.cpp
  PlatformAlertRecord.cpp
  RangeMark.cpp
)

SET(HEADERS
  CMAlert.h
  ContactMgrCore.h
  PlatformAlertRecord.h
  RangeMark.h
)

# Build Library
IVP_ADD_LIBRARY(contactmgr ${SRC})
TARGET_LINK_LIBRARIES(contactmgr
  PUBLIC bhvutil contacts geodaid geometry mbutil)
