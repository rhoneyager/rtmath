include (common)

set(batch-runs_mocheaders dispatcher.h worker.h)
#set(batch-runs_uis dispatcher.ui)
#set(batch-runs_rcs dispatcher.qrc)

set(QT_DONT_USE_QTGUI true)

find_package( Qt4 COMPONENTS QtCore REQUIRED )

INCLUDE( ${QT_USE_FILE} )

#QT4_ADD_RESOURCES( batch-runs_RC_SRCS ${batch-runs_rcs} )

#QT4_WRAP_UI ( batch-runs_UI_HDRS ${batch-runs_uis} )

QT4_WRAP_CPP ( batch-runs_MOC_SRCS ${batch-runs_mocheaders} )

include_directories (
	${CMAKE_BINARY_DIR}
)

include_directories ( BEFORE SYSTEM 
	${QT_INCLUDE_DIR}
#	/usr/include/QtGui
)

add_executable (tmatrix-batch-runs
	main.cpp
	dispatcher.h dispatcher.cpp
	worker.h worker.cpp
	comm.h comm.cpp
	debug.h debug.cpp
	${batch-runs_MOC_SRCS}
#	${batch-runs_RC_SRCS}
#	${batch-runs_UI_HDRS}
	)

addbasicapp(tmatrix-batch-runs)
if(LOAD_PACKAGES)
	add_package_dependency(tmatrix-batch-runs DEPENDS_ON Boost)
	target_link_libraries (tmatrix-batch-runs ${tmatrix-batch-runs_LIBRARIES})
	target_link_libraries (tmatrix-batch-runs boost_filesystem boost_program_options boost_serialization boost_iostreams boost_unit_test_framework boost_system)
endif()
TARGET_LINK_LIBRARIES( tmatrix-batch-runs ${QT_LIBRARIES} tmatrix-cpp tmatrix-fortran)

