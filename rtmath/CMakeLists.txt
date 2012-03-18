# CMake solution for the rtmath library
# This conveniently can produce unix makefiles, eclipse .project files, 
# visual studio solutions and xcode projects

cmake_minimum_required (VERSION 2.6)
project(rtmath)

# Load some basic macros which are needed later on
#include(FindROOT.cmake)

# Recurse into the subdirectories

add_subdirectory (rtmath)
add_subdirectory (rtmath-apps)
#add_subdirectory (tests)
