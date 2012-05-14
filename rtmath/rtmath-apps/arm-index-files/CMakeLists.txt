include (common)

add_executable (rtmath-arm-index-files main.cpp)
        find_package(PostgreSQL REQUIRED)
        find_package(PQXX REQUIRED)

addapp(rtmath-arm-index-files)
add_package_dependency(rtmath-arm-index-files DEPENDS_ON PQXX)
add_package_dependency(rtmath-arm-index-files DEPENDS_ON PostgreSQL)
target_link_libraries (rtmath-arm-index-files pq pqxx)


