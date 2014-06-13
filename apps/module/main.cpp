#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/modules.h"


int main(int argc, char **argv)
{
	try {
		using namespace std;
		using namespace Ryan_Debug;
		namespace po = boost::program_options;
		po::positional_options_description p;
		po::options_description cmdline("Command-line options"), config("Config options"),
			hidden("Hidden options"), desc("Allowed options"), oall("all options");
		Ryan_Debug::add_options(cmdline, config, hidden);
		cmdline.add_options()
			("help,h", "produce help message")
			("cmake", po::value<vector<string> >(), "Set cmake directory for product")
			("lib", po::value<vector<string> >(), "Set library directories")
			("bin", po::value<vector<string> >(), "Set binary directories")
			("include", po::value<vector<string> >(), "Set include directories")
			("prereq", po::value<vector<string> >(), "Specify prerequisite groups")
			("groupname", po::value<string>(), "Manually specify the group name (otherwise determined from target name)")
			("name", po::value<string>(), "Manually specify the product name (otherwise determined from target name)")
#ifdef _WIN32
			("cygwin-base", po::value<string>(), "Set cygwin base directory")
#endif
			("module-base", po::value<string>(), "Set base directory for modulefiles")
			("operation", po::value<string>(), "Specify action to take - "
			"create - creates a new module file matching the target information. If it exists, do nothing. "
			"update - create or update a module file. "
			//"erase - delete a matching module file (searches by key). "
			//"link - updates a module-containing directory with symlinks to the latest version for each tag. "
			//"default - set a given module file as a system default. "
			)
			("target", po::value<string>(), "Specify target file")
			("private", po::value<bool>()->default_value(0), "Is this a private module?")
			;
		p.add("operation", 1);
		p.add("target", 2);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(oall).
			positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << message << endl;
			exit(1);
		};
		if (vm.count("help")) doHelp("");
		Ryan_Debug::process_static_options(vm);

		if (!vm.count("operation")) doHelp("Need to specify an operation.");
		string op = vm["operation"].as<string>();
		if (!vm.count("target")) doHelp("Need to specify a target.");
		string starget = vm["target"].as<string>();

		using boost::filesystem::path;
		multimap<string, path> paths;
		auto addPaths = [&](const std::string &key, bool multi, bool required, bool cygrel)
		{
			if (!vm.count(key))
			{
				if (required) doHelp("Required path not specified.");
				else return;
			}
			vector<string> p = vm[key].as<vector<string> >();
			if (!multi && p.size() > 1) doHelp("Single value allowed for path.");
			for (const auto &i : p)
			{
				path pi(i);
				path pabs = boost::filesystem::absolute(pi);
				if (cygrel)
					pabs = Ryan_Debug::convertUnix(pabs);

				paths.insert(std::pair<string, path>(key, pabs));
			}
		};
		addPaths("cmake", false, false, false);
		addPaths("lib", true, false, true);
		addPaths("bin", true, false, true);
		addPaths("include", true, false, true);
		vector<string> prereqs;
		if (vm.count("prereq")) prereqs = vm["prereq"].as<vector<string> >();

		// Figure out the target path
		path moduleBase;
		if (vm.count("module-base"))
		{
			moduleBase = path(vm["module-base"].as<string>());
		} else {
			bool isPrivate = vm["private"].as<bool>();
#ifdef _WIN32
			// Get cygwin base directory from command-line or registry
			path cygbase;
			if (vm.count("cygwin-base"))
			{
				string scbase = vm["cygwin-base"].as<string>();
				cygbase = path(scbase);
			}
			else {
				// Key is at HKLM\SOFTWARE\Cygwin\setup\rootdir [C:\cygwin64]
				bool res = findCygwinBaseDir(cygbase);
				if (!res) doHelp("Cannot find Cygwin base directory.");
			}

			if (!isPrivate)
				moduleBase = cygbase / path("etc") / path("modulefiles");
			else {
				// Figure out username and home directory
				doHelp("Cannot determine cygwin home directory.");
			}
#else
			if (!isPrivate)
				moduleBase = path("/etc/modulefiles");
			else
			{
				path pHome;
				if (!findUserHomeDir(pHome)) doHelp("Cannot find user home directory.");
				moduleBase = pHome;
				moduleBase /= path("privatemodules");
			}
#endif
		}

		if (op == "create" || op == "update")
		{
			if (!vm.count("target")) doHelp("Target is required for this operation.");
			path pTargetCand = path(vm["target"].as<string>());
			path pTarget;
			if (!pTargetCand.is_absolute()) pTarget = moduleBase / pTargetCand;
			else pTarget = pTargetCand;

			// Determine group name by chopping off pTarget through moduleBase, then taking the first element.
			string groupName;
			string cmakename;
			if (vm.count("groupname")) groupName = vm["groupname"].as<string>();
			else {
				if (pTargetCand.is_absolute()) doHelp("Need to manually specify 'groupname' in this case");
				auto it = moduleBase.begin();
				auto ot = pTarget.begin();
				while (it != moduleBase.end()) { ++it; ++ot; }
				groupName = ot->string();
			}
			if (vm.count("name")) cmakename = vm["name"].as<string>();
			else if (groupName.size()) cmakename = groupName;
			else doHelp("Need to manually specify product 'name' in this case");

			if (exists(pTarget) && op == "create") doHelp("Target file exists, and create-only mode is specified.");
			// Create the folder tree
			path pFld = pTarget;
			pFld.remove_filename();
			if (!exists(pFld))
				boost::filesystem::create_directories(pFld);
			
			// Write the file
			ofstream out(pTarget.string().c_str());
			out << "#%Module 1.0" << std::endl;
			if (prereqs.size())
			{
				for (const auto &p : prereqs)
					out << "prereq\t\t" << p << std::endl;
			}
			out << "\nconflict\t\t"
				<< groupName << std::endl
				<< std::endl;
			auto escString = [](const path &p) -> std::string
			{
				std::string str = p.generic_string();
				const std::string from(" "), to("\\ ");

				//void replaceAll(std::string& str, const std::string& from, const std::string& to) {
				if (from.empty())
					return str;
				size_t start_pos = 0;
				while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
					str.replace(start_pos, from.length(), to);
					start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
				}

				return str;
			};

			for (const auto &p : paths)
			{
				if (p.first == "cmake") {
					out << "setenv\t" << cmakename << "_DIR\t" << escString(p.second) << std::endl;
				}
				else if (p.first == "bin") {
					out << "prepend-path\tPATH\t" << escString(p.second) << std::endl;
				}
				else if (p.first == "lib") {
					out << "prepend-path\tLD_LIBRARY_PATH\t" << escString(p.second) << std::endl;
					out << "prepend-path\tLIBRARY_PATH\t" << escString(p.second) << std::endl;
				}
				else if (p.first == "include") {
					out << "prepend-path\tINCLUDE\t" << escString(p.second) << std::endl;
					out << "prepend-path\tC_INCLUDE_PATH\t" << escString(p.second) << std::endl;
					out << "prepend-path\tCPLUS_INCLUDE_PATH\t" << escString(p.second) << std::endl;
				}
				else
				{
					std::cerr << "Unknown key " << p.first << std::endl;
					std::cerr << "Near line " << __LINE__ << std::endl;
					doHelp("Fix coding logic");
				}
			}
		}
		else doHelp("Unrecognized operation.");
	}
	catch (std::exception & e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}

