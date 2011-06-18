// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently

#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <set>
#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <time.h>
#include <exception>
#include <new>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "debug.h"
//#include "error.h"
//#include "damatrix.h"

// The heap-checking overridess to new and delete, to find bugs!!!


#ifdef HEAP_CHECK

//void AddTrack(void* addr,  size_t asize,  const char *fname, int lnum);
//void RemoveTrack(void* addr);

inline void* operator new(size_t size, const char *file, int line)
{
	void *p = malloc(size);
	if (p==0) // Fail
		throw std::bad_alloc();
	//AddTrack(p, size, file, line);
	std::cerr << "New called with size " << size << std::endl;
	std::cerr << " given memory starting at " << p << std::endl;
	//memlog newmem(p,size);
	//std::cerr << " assigning id " << newmem.id << std::endl;
	//memmaps.insert(newmem);
	return p;
}

inline void operator delete(void *p)
{
	std::cerr << "Delete called on memory " << p << std::endl;
	//RemoveTrack(p);
	//std::cerr << "This was id " << std::endl;
	free(p);
}

inline void* operator new[](size_t size, const char *file, int line)
{
	void *p = (void*) malloc(size);
	if (p==0) // Fail
		throw std::bad_alloc();
	//AddTrack(p, size, file, line);
	std::cerr << "New called with size " << size << std::endl;
	std::cerr << " given memory starting at " << p << std::endl;
	//memlog newmem(p,size);
	//std::cerr << " assigning id " << newmem.id << std::endl;
	//memmaps.insert(newmem);
	return p;
}

inline void operator delete[](void *p)
{
	std::cerr << "Delete called on memory " << p << std::endl;
	//RemoveTrack(p);
	//std::cerr << "This was id " << std::endl;
	free(p);
}
#endif



#ifdef HEAP_CHECK

#include <exception>
#include <new>
#include <cstdlib>
#include <set>
//#define DEBUG_NEW new(__FILE__, __LINE__)

#else

//#define DEBUG_NEW new

#endif

//#define new DEBUG_NEW
