/* debug_mem_class.h
 * Include this file in a class to enable high-level memory debugging.
 * Remember to include the necessary other headers in the calling header!
 * These are cstdlib, iostream, set
 */

#ifdef SPECIAL_HEAP_CHECK

//void AddTrack(void* addr,  size_t asize,  const char *fname, int lnum);
//void RemoveTrack(void* addr);

static const char* __file__;
static size_t __line__;
static const char* __caller__;

static void __Track(int option, void* p, size_t size, const char* file, int line, const char* caller)
{
	// General tracking function
	// For convenience right now, save all in static vars
	// Calling structure:
	//  1 (insert), takes all arguments
	//  2 (remove), takes pointer
	//  3 (list), lakes none. Lists all remaining data
	// Since this is defined before new and new[], I can use stl sets without extra noise
	// but, these sets will be vulnerable to their own memory corruption!
	// However, the data will still be listed
	using namespace std;
	static std::map<void*, size_t> sizes;
	static std::map<void*, const char*> files;
	static std::map<void*, const char*> callers;
	static std::map<void*, int> lines;

	switch (option)
	{
	case 1:
		// Add assignment
		sizes[p] = size;
		files[p] = file;
		lines[p] = line;
		callers[p] = caller;
		break;
	case 2:
		// Delete assignment
		if (sizes.count(p) == 0) fprintf(stderr, "\nError! Pointer delete without allocation: %p\n", p);
		sizes.erase(p);
		files.erase(p);
		lines.erase(p);
		callers.erase(p);
		break;
	case 3:
		// List current assignments
		fprintf(stderr, "\nListing remaining memory assignments:\n");
		fprintf(stderr, "Size: %d\n", sizes.size());
		fprintf(stderr, "Pointer range\tSize\tFile\tLine\tCaller\n");
		for (std::map<void*,size_t>::const_iterator it=sizes.begin(); it!=sizes.end(); it++)
		{
			int *end = &((int*) it->first)[it->second / sizeof(int)];
			// I'll use just one iterator. The key is the same in each map
			fprintf(stderr, "%p - %p\t%d\t%s\t%d\t%s\n", it->first, end, it->second, 
				files[it->first], lines[it->first], callers[it->first]);
		}
		fprintf(stderr,"\nEnd of remaining callers.\n");
		break;
	default:
		break;
	}
}

inline void* operator new(size_t size)
{
	void *p = malloc(size);
	if (p==0) // Fail
		throw std::bad_alloc();
	__Track(1,p,size,__file__,__line__,__caller__);
	//AddTrack(p, size, file, line);
	printf("\n\n");
	if (__caller__) fprintf(stderr,"HEAP: new called from: %s\n", __caller__);
	if (__file__) fprintf(stderr, " file: %s\n", __file__);
	if (__line__) fprintf(stderr, " line: %d\n", __line__);
	fprintf(stderr, " size: %d\n", size);
	int lengthInt = size / sizeof(int);
	int *end = &(((int*) p)[lengthInt]);
	fprintf(stderr, " memory range: %p - %p\n", p, end);
	__file__ = 0;
	__line__ = 0;
	__caller__ = 0;
	return p;
}

inline void* operator new[](size_t size)
{
	void *p = malloc(size);
	if (p==0) // Fail
		throw std::bad_alloc();
	__Track(1,p,size,__file__,__line__,__caller__);
	//AddTrack(p, size, file, line);
	printf("\n\n");
	if (__caller__) fprintf(stderr,"HEAP: new called from: %s\n", __caller__);
	if (__file__) fprintf(stderr, " file: %s\n", __file__);
	if (__line__) fprintf(stderr, " line: %d\n", __line__);
	fprintf(stderr, " size: %d\n", size);
	int lengthInt = size / sizeof(int);
	int *end = &(((int*) p)[lengthInt]);
	fprintf(stderr, " memory range: %p - %p\n", p, end);
	__file__ = 0;
	__line__ = 0;
	__caller__ = 0;
	return p;
}

inline void operator delete(void *p)
{
	printf("Delete called on memory %p\n", p);
	__Track(2,p,0,0,0,0);
	//RemoveTrack(p);
	__file__ = 0;
	__line__ = 0;
	__caller__ = 0;
	free(p);
}

inline void operator delete[](void *p)
{
	delete(p);
}

static bool setloc(const char* _file, int _line, const char* _caller)
{
	__file__ = _file;
	__line__ = _line;
	__caller__ = _caller;
	return false;
}

// End special heap check code
#endif

#ifdef SPECIAL_HEAP_CHECK

//#define new (__file__=__FILE__, __line__=__LINE__, __caller__=__FUNCSIG__) && 0 ? NULL : new

#ifdef __GNUC__
//#define DEBUG_NEW new(__FILE__, __LINE__, __PRETTY_FUNCTION__)
#define new (setloc(__FILE__,__LINE__,__PRETTY_FUNCTION__)) ? NULL : new
define dnew (__file=__FILE__,__line__=__LINE__,__caller__=__PRETTY_FUNCTION__) && 0 ? NULL : new
#endif
#ifdef _MSC_FULL_VER
//#define DEBUG_NEW new(__FILE__, __LINE__, __FUNCSIG__)
#define new (setloc(__FILE__,__LINE__,__FUNCSIG__)) ? NULL : new
//#define dnew (__file__=__FILE__, __line__=__LINE__, __caller__=__FUNCSIG__) && 0 ? NULL : new
#endif

//#define new dnew

#endif

