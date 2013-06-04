include (common)
set(rtmath-tmatrix-mie-test-qt_LIBRARIES "")
set(rtmath-tmatrix-mie-test-qt_INCLUDE_DIRS "")


set(mie-test-qt_mocheaders frmmain.h)
set(mie-test-qt_uis frmmain.ui)

find_package( Qt4 COMPONENTS QtCore QtGui REQUIRED )
#find_package( Qt4 REQUIRED QtCore QtGui )
#message("${QT_LIBRARIES}")
#message("${QT_INCLUDES}")

QT4_WRAP_UI ( mie-test-qt_UI_HDRS ${mie-test-qt_uis} )

QT4_WRAP_CPP ( mie-test-qt_MOC_SRCS ${mie-test-qt_mocheaders} )

include_directories (
	${CMAKE_BINARY_DIR}
)

add_executable (rtmath-tmatrix-mie-test-qt
	main.cpp frmmain.cpp
	${mie-test-qt_MOC_SRCS}
	${mie-test-qt_UI_HDRS}
	)

addapp(rtmath-tmatrix-mie-test-qt)
INCLUDE( ${QT_USE_FILE} )
TARGET_LINK_LIBRARIES( rtmath-tmatrix-mie-test-qt ${QT_LIBRARIES} tmatrix-cpp tmatrix-fortran tmatrix-serialization)

