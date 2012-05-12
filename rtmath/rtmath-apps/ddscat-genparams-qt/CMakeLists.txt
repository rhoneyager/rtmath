include (common)

set(genparams-qt_mocheaders frmmain.h)
set(genparams-qt_uis frmmain.ui)
set(genparams-qt_rcs frmmain.qrc)

find_package( Qt4 REQUIRED )

INCLUDE( ${QT_USE_FILE} )

QT4_ADD_RESOURCES( genparams-qt_RC_SRCS ${genparams-qt_rcs} )

QT4_WRAP_UI ( genparams-qt_UI_HDRS ${genparams-qt_uis} )

QT4_WRAP_CPP ( genparams-qt_MOC_SRCS ${genparams-qt_mocheaders} )

INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR} )

add_executable (rtmath-ddscat-genparams-qt 
	main.cpp stdafx.cpp frmmain.cpp
	${genparams-qt_MOC_SRCS}
	${genparams-qt_RC_SRCS}
	${genparams-qt_UI_HDRS}
	)

addapp(rtmath-ddscat-genparams-qt)
TARGET_LINK_LIBRARIES( rtmath-ddscat-genparams-qt ${QT_LIBRARIES} )

