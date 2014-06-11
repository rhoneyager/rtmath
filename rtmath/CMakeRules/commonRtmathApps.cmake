

find_package(Ryan_Debug REQUIRED)
include_directories(${RYAN_DEBUG_INCLUDE_DIRS})
find_package(Ryan_Serialization REQUIRED)
include_directories(${RYAN_SERIALIZATION_INCLUDE_DIRS})

# Pulled in much earlier
#addBoostUniform(program_options serialization iostreams filesystem system unit_test_framework regex)
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

