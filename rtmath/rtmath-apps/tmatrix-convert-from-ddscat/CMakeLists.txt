include (common)
set(rtmath-tmatrix-convert-from-ddscat-qt_LIBRARIES "")
set(rtmath-tmatrix-convert-from-ddscat-qt_INCLUDE_DIRS "")


set(tmconvert-qt_mocheaders frmmain.h)
set(tmconvert-qt_uis frmmain.ui)

find_package( Qt4 COMPONENTS QtCore QtGui REQUIRED )
#find_package( Qt4 REQUIRED QtCore QtGui )
#message("${QT_LIBRARIES}")
#message("${QT_INCLUDES}")

QT4_WRAP_UI ( tmconvert-qt_UI_HDRS ${tmconvert-qt_uis} )

QT4_WRAP_CPP ( tmconvert-qt_MOC_SRCS ${tmconvert-qt_mocheaders} )

include_directories (
	${CMAKE_BINARY_DIR}
)

add_executable (rtmath-tmatrix-convert-from-ddscat-qt 
	main.cpp frmmain.cpp converter.cpp converter2.cpp
	${tmconvert-qt_MOC_SRCS}
	${tmconvert-qt_UI_HDRS}
	)

#add_package_dependency(rtmath-tmatrix-convert-from-ddscat-qt DEPENDS_ON ROOT)

addapp(rtmath-tmatrix-convert-from-ddscat-qt)
INCLUDE( ${QT_USE_FILE} )
TARGET_LINK_LIBRARIES( rtmath-tmatrix-convert-from-ddscat-qt ${QT_LIBRARIES} tmatrix-cpp tmatrix-fortran tmatrix-serialization)

