include (common)
set(rtmath-shape-basicdata_LIBRARIES "")
set(rtmath-shape-basicdata_INCLUDE_DIRS "")

add_executable (rtmath-shape-basicdata main.cpp)

add_package_dependency(rtmath-shape-basicdata DEPENDS_ON ROOT)
set(rtmath-shape-basicdata_LIBRARIES "${rtmath-shape-basicdata_LIBRARIES};Spectrum")

addapp(rtmath-shape-basicdata)
#message("${rtmath-shape-basicdata_LIBRARIES}")

