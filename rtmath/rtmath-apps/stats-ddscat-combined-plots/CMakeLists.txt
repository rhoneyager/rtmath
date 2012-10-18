include (common)
set(rtmath-stats-ddscat-combined-plots_LIBRARIES "")
set(rtmath-stats-ddscat-combined-plots_INCLUDE_DIRS "")

add_executable (rtmath-stats-ddscat-combined-plots main.cpp)

add_package_dependency(rtmath-stats-ddscat-combined-plots DEPENDS_ON ROOT)

addapp(rtmath-stats-ddscat-combined-plots)

