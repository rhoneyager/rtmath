include (common)
set(rtmath-ddscat-ddpar_LIBRARIES "")
set(rtmath-ddscat-ddpar_INCLUDE_DIRS "")

add_executable (rtmath-ddscat-ddpar main.cpp)

add_package_dependency(rtmath-ddscat-ddpar DEPENDS_ON ROOT)

addapp(rtmath-ddscat-ddpar)

