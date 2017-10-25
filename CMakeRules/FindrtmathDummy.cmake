# - Find rtmath dummy library
#

SET( RTMATH_FOUND TRUE )
SET( RTMATHDUMMY_FOUND TRUE)

SET( rtmathDummy_INCLUDE_DIRS ${rtmath_INCLUDES})
SET( rtmathDummy_LIBRARIES ${rtmath_LIBS})
MARK_AS_ADVANCED( rtmath-static_DIR rtmath_DIR rtmathDummy_DIR)

