#include "debug_mem.h"
#include "Stdafx.h"

namespace rtmath {
	namespace debug {
		namespace memcheck {
			const char* __file__;
			size_t __line__;
			const char* __caller__;
			bool enabled = true;
			/*
			bool setloc(const char* _file, int _line, const char* _caller)
			{
				using namespace rtmath::debug::memcheck;
				__file__ = _file;
				__line__ = _line;
				__caller__ = _caller;
				return false;
			}
			*/
			bool __Track(int option, void* p, size_t size, const char* file, int line, const char* caller)
			{
				//if (option == 1 && file == 0) return; // Sanity check
				if (!enabled) return false;
				if (file)
					if (strstr(file,"debug_mem.cpp")) return false; // Do not trigger!
				
				enabled = false; // Lock __Track
				// General tracking function
				// For convenience right now, save all in static vars
				// Calling structure:
				//  1 (insert), takes all arguments
				//  2 (remove), takes pointer
				//  3 (list), lakes none. Lists all remaining data
				// Since this is defined before new and new[], I can use stl maps without extra noise
				// but, these maps will be vulnerable to their own memory corruption!
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
				enabled = true; // Unlock __Track
				return true;
			}

		};
	};
};
