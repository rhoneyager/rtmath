include (common)

set (srcs
	ddOutputSingleTrees.cpp
	ddOutputSingleTrees.h
	LinkDef.h
	main.cpp
	out.cxx
	out.h
	)

add_executable (ROOTtest ${srcs})
addapp(ROOTtest)

