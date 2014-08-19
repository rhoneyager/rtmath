include (common)
include (commonRtmathApps)
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/"
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
#set(PostgreSQL_ADDITIONAL_VERSIONS "9.3")
find_package(PostgreSQLrtmath REQUIRED)
#message("Include ${psql_INCLUDE_DIRS}")
include_directories(${PostgreSQL_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


set (sql_files 
	tables/arm_tables.sql
	tables/dda_run_tables.sql
	tables/flakes.sql
	)
source_group("SQL" FILES ${sql_files})

add_library (rtmath_plugin_psql SHARED 
	#nc-arm_info.cpp
	#nc-sacr.cpp
	plugin-psql.cpp
	plugin-psql.h
	psql-arm-info.cpp

	${sql_files}
	)
addlib(rtmath_plugin_psql SHARED)
#message("${PostgreSQL_INCLUDE_DIRS} Libs ${PostgreSQL_LIBRARIES}")

target_link_libraries(rtmath_plugin_psql 
	${rtmath_data-libs} 
	rtmath_core 
	rtmath_voronoi
	rtmath_data
	${PostgreSQL_LIBRARIES})
addplugin(rtmath_plugin_psql RT)

