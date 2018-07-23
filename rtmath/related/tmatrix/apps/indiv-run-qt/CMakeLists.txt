include (common)

set(indiv-run-qt_mocheaders frmmain.h)
set(indiv-run-qt_uis frmmain.ui)
set(indiv-run-qt_rcs frmmain.qrc)

SET(QT_USE_QTOPENGL TRUE)

find_package( Qt4 COMPONENTS QtCore QtGui REQUIRED )

INCLUDE( ${QT_USE_FILE} )

QT4_ADD_RESOURCES( indiv-run-qt_RC_SRCS ${indiv-run-qt_rcs} )

QT4_WRAP_UI ( indiv-run-qt_UI_HDRS ${indiv-run-qt_uis} )

QT4_WRAP_CPP ( indiv-run-qt_MOC_SRCS ${indiv-run-qt_mocheaders} )

include_directories (
	${CMAKE_BINARY_DIR}
)

include_directories ( BEFORE SYSTEM 
	${QT_INCLUDE_DIR}
#	/usr/include/QtGui
)

add_executable (tmatrix-indiv-run-qt 
	main.cpp frmmain.cpp frmmain.h
	${indiv-run-qt_MOC_SRCS}
	${indiv-run-qt_RC_SRCS}
	${indiv-run-qt_UI_HDRS}
	)

addbasicapp(tmatrix-indiv-run-qt)

TARGET_LINK_LIBRARIES( tmatrix-indiv-run-qt boost_serialization ${QT_LIBRARIES} tmatrix-cpp tmatrix-fortran)

