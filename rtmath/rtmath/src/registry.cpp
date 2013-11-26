/**
* \brief Contains registry functions for extending functionality through DLLs.
*
* Contains both general and OS-specific functions.
**/

#include "Stdafx-core.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <Ryan_Debug/debug.h>
#include "../rtmath/error/debug.h"
#include "../rtmath/error/debug_mem.h"
#include "../rtmath/error/error.h"
#include "../rtmath/registry.h"

#ifdef _WIN32
#include "windows.h"
#endif
#ifdef __DLSYM__
#include "dlfcn.h"
#endif

#ifdef _WIN32
typedef HINSTANCE dlHandleType;
#endif
#ifdef __APPLE__
typedef void* dlHandleType;
#endif
#ifdef __unix__
typedef void* dlHandleType;
#endif

/// DLL information tables
namespace {
	class DLLhandle;
	std::set<std::string> DLLpathsLoaded;
	std::vector<rtmath::registry::DLLpreamble> preambles;
	std::vector<boost::shared_ptr<DLLhandle> > handles;

	std::vector<boost::filesystem::path> searchPaths;
	//bool autoLoadDLLs = true;

	typedef std::map<std::string, rtmath::registry::classHookMapType > hookRegistryType;
	hookRegistryType hookRegistry;

	void loadSearchPaths()
	{
		throw rtmath::debug::xUnimplementedFunction();
	}

	/** \brief Class that loads the DLL in an os-independent manner.
	*
	* Construction is allocation.
	* This class enables the safe closiong of dlHandle if registration 
	* fails to initialize properly. Previously, a dangling handle was
	* produced.
	**/
	class DLLhandle
	{
	public:
		DLLhandle(const std::string &filename) : dlHandle(nullptr)
		{
			open(filename);
		}
		DLLhandle() : dlHandle(nullptr)
		{
		}
		void open(const std::string &filename)
		{
			if (DLLpathsLoaded.count(filename)) throw rtmath::debug::xDuplicateHook(fname.c_str());
			if (dlHandle) throw rtmath::debug::xHandleInUse(fname.c_str());
			fname = filename;
#ifdef __DLSYM__ // Indicates that DLSYM is provided (unix, linux, mac, etc. (sometimes even windows))
			//Check that file exists here
			this->dlHandle = dlopen(filename.c_str(), RTLD_LAZY);
			if (dlerror()) throw xFileNotFound(filename);
#endif
#ifdef _WIN32
			this->dlHandle = LoadLibrary(filename.c_str());
			if (this->dlHandle == NULL) throw rtmath::debug::xMissingFile(filename.c_str());
#endif
			DLLpathsLoaded.insert(filename);
		}
		void close()
		{
			if (!dlHandle) return;
			DLLpathsLoaded.erase(fname);
#ifdef __unix__
			dlclose(this->dlHandle);
#endif
#ifdef _WIN32
			FreeLibrary(this->dlHandle);
#endif
		}
		~DLLhandle()
		{
			if (dlHandle) close();
		}
		void* getSym(const char* symbol)
		{
			if (dlHandle == NULL) throw rtmath::debug::xHandleNotOpen(fname.c_str());
			void* sym;
#ifdef __DLSYM__
			sym = dlsym(dlHandle, symbol);
#endif
#ifdef _WIN32
			sym = GetProcAddress(dlHandle, symbol);
#endif
			if (!sym) throw rtmath::debug::xSymbolNotFound(symbol, fname.c_str());
			return (void*)sym;
		}
	private:
		std::string fname;
		dlHandleType dlHandle;
	};
}

namespace rtmath
{
	namespace registry
	{

		void add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden)
		{
			namespace po = boost::program_options;
			using std::string;

			cmdline.add_options()
				;

			config.add_options()
				;

			hidden.add_options()
				("dll-load", po::value<std::vector<std::string> >(),
				"Specify dlls to load. If passed a directory, it loads all dlls present (one-level). "
				"Will use search paths in name resolution.")
			//	("dll-no-default-locations", "Prevent non-command line dll locations from being read")
				("print-dll-loaded", "Prints the table of loaded DLLs.")
				("print-dll-search-paths", "Prints the search paths used when loading dlls.")
				;
		}

		void process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;

			//if (vm.count("dll-no-default-locations"))
			//	autoLoadDLLs = false;

			if (vm.count("dll-load"))
			{
				using namespace boost::filesystem;
				std::vector<std::string> sPaths = vm["dll-load"].as<std::vector<std::string> >();
				for (const auto s : sPaths)
				{
					path op(s);

					// Attempt to find the path if not absolute
					if (!op.is_absolute())
					{
						auto search = [&](const path& op, path& res) -> bool
						{
							for (const path &sp : searchPaths)
							{
								if (exists(sp / op))
								{
									res = sp / op;
									return true;
								}
							}
							return false;
						};
						if (!search(op, op)) throw debug::xMissingFile(op.string().c_str());
					}

					// Expand symlinks
					path p = debug::expandSymlink(op);
					if (is_directory(p))
					{
						// Open all libs under one level of this directory, following symlinks
						std::vector<path> cands;
						copy(directory_iterator(p),
							directory_iterator(), back_inserter(cands));
						for (auto f = cands.begin(); f != cands.end(); ++f)
						{
							path pf = *f;
							pf = debug::expandSymlink(pf);
							path pext = pf.extension();
							if (pext.string() == ".dll" || pf.string().find(".so") != string::npos)
							{
								loadDLL(pf.string());
							}
						}
					} else {
						if (exists(p))
							loadDLL(p.string());
						else {
							// Search for matching files in the same directory with the dll or so extension.
							// TODO
							throw debug::xMissingFile(p.string().c_str());
						}
					}
				}
			}

			if (vm.count("print-dll-search-paths"))
				printDLLsearchPaths();

			if (vm.count("print-dll-loaded"))
				printDLLs();
		}

		void loadDLL(const std::string &filename)
		{
			auto doLoad = [](const std::string &f)
			{
				boost::shared_ptr<DLLhandle> h(new DLLhandle(f));
				handles.push_back(h);
			};
			// Search for the dll
			using namespace boost::filesystem;
			path p(filename);
			if (p.is_absolute())
			{
				if (exists(p)) doLoad(p.string());
				else throw debug::xMissingFile(p.string().c_str());
			}
			else {
				// Load in the default search paths
				if (!searchPaths.size())
					loadSearchPaths();
				// Proceed through the search paths to attempt to find the file
				for (const auto &s : searchPaths)
				{
					path pr = boost::filesystem::absolute(p, s);
					if (exists(pr))
					{
						doLoad(pr.string());
						return;
					}
				}
				// By this point, the path cannot be found
				throw debug::xMissingFile(p.string().c_str());
			}
			
		}

		void printDLLs(std::ostream &out)
		{
			out << "rtmath DLL registry table:\n--------------------\n";
			for (const auto p : preambles)
			{
				out << "Name:\t" << p.name << "\n"
					<< "Description:\t" << p.description << "\n"
					<< "UUID:\t" << p.uuid << "\n";
				if (p.path)
					out << "Path:\t" << p.path << std::endl;
			}
			out << std::endl;
			out << "DLL paths loaded:\n----------------\n";
			for (const auto p : DLLpathsLoaded)
				out << p << "\n";
			out << std::endl;

			out << "\nHook Table:\n-----------------\nClass\tTopic\tPointer";
			for (const auto hm : hookRegistry)
			{
				out << hm.first << "\n";
				for (const auto h : hm.second)
				{
					out << "\t" << h.first << "\t" << h.second << "\n";
				}
			}

			out << std::endl;
		}

		void printDLLsearchPaths(std::ostream &out)
		{
			out << "rtmath DLL registry search paths:\n--------------------\n";
			for (const auto p : searchPaths)
			{
				out << p.string() << "\n";
			}
			out << std::endl;
		}

		void queryClass(const char* classname, classHookMapType& result)
		{
			result.clear();
			std::string shc(classname);
			if (hookRegistry.count(shc))
			{
				result = hookRegistry.at(shc);
			}
		}
	}
}

extern "C"
{
	bool rtmath_registry_register_dll(const rtmath::registry::DLLpreamble &p)
	{
		rtmath::registry::DLLpreamble b = p;
		// \todo Add in dll path to preamble!
		preambles.push_back(b);
		return true;
	}

	bool rtmath_registry_register_hook(const char* hookedClass, const char* topic, void* func)
	{
		// \todo Replace void* with an appropriate function definition

		std::string shc(hookedClass);
		if (!hookRegistry.count(shc))
		{
			rtmath::registry::classHookMapType newHookMap;
			hookRegistry.insert(std::pair < std::string, rtmath::registry::classHookMapType >
				(shc, newHookMap));
		}

		rtmath::registry::classHookMapType &map = hookRegistry.at(shc);

		map.insert(std::pair<std::string, void*>(topic, func));
	}

	
}

