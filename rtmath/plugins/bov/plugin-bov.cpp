/// \brief Provides bov file reading and writing to the shapefile class

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/error/debug.h"

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

void dllEntry();

// DLL binding and unbinding code
#ifndef _MSC_FULL_VER //__GNUC__, __llvm__, __clang__, __INTEL_COMPILER, ...

/// GCC - code that is executed on DLL import
void __attribute__((constructor)) plugin_bov_gcc_init()
{
	dllEntry();
}

/// GCC - code that is executed on DLL unload
void __attribute__((destructor)) plugin_bov_gcc_fini()
{
}
#endif
#ifdef _MSC_FULL_VER
/// MSVC - code that is executed on DLL load and unload
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
	if (dwReason == DLL_PROCESS_ATTACH)
	{
		dllEntry();
		// DisableThreadLibraryCalls(hInstance);
	}
	return true;
}
#endif

void dllEntry()
{
	std::cerr << "plugin-bov dll loaded!\n\n";
}


/*
int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-extract\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< string >(), "input shape file")
			("bov,b", po::value<string>(), "output bov file prefix")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size()) cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		if (!vm.count("input")) doHelp("Need to specify an input file.");
		string input = vm["input"].as<string>();
		cerr << "Reading input shape file " << input << endl;
		rtmath::ddscat::shapefile shp;
		shp.read(input);

		if (vm.count("bov"))
		{
			string bPrefix = vm["bov"].as<string>();
			cerr << "Writing BOV files with prefix " << bPrefix << endl;
			//shp.write(string(bPrefix).append("-orig.dat"));
			shp.writeBOV(bPrefix);
		}

	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}
*/
