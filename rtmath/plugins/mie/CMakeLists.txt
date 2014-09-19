include (common)
include (commonRtmathApps)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_library (rtmath_plugin_mie SHARED 
	plugin-mie.cpp
	plugin-mie.h
	mie.h
	mie-abNCalc.h
	mie-AnCalc.h
	mie-piNCalc.h
	mie-Qcalc.h
	mie-Scalc.h
	mie-tauNCalc.h
	mie-wnCalc.h
	mie.cpp
	mie-abNCalc.cpp
	mie-AnCalc.cpp
	mie-piNCalc.cpp
	mie-Qcalc.cpp
	mie-Scalc.cpp
	#mie-serialization.cpp
	mie-tauNCalc.cpp
	mie-wnCalc.cpp
	)
addlib(rtmath_plugin_mie SHARED)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


target_link_libraries(rtmath_plugin_mie ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat )
addplugin(rtmath_plugin_mie non_dda)

