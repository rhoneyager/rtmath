
# Pulled in much earlier
#addBoostUniform(program_options serialization iostreams filesystem system unit_test_framework regex)
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")

