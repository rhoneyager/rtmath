#include <iostream>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/info.h"
#include "Ryan.Debug.DebugAssembly.manifest.h"

/// This application provides recursive process information for the specified pid.
/// If no pid is specified, starts at the current process.
/// Recurses until 'bash', 'screen', 'tcsh', 'at', or 'sshd' is found.
/// Also can incorporate a line from the invoker into the output (useful for scripting).
int main(int argc, char **argv)
{
	try {
		using namespace std;
		using namespace Ryan_Debug;
		namespace po = boost::program_options;
		po::positional_options_description p;
		p.add("pid", 1);
		p.add("message", -1);
		po::options_description cmdline("Command-line options"), config("Config options"), 
			hidden("Hidden options"), desc("Allowed options"), oall("all options");
		Ryan_Debug::add_options(cmdline, config, hidden);
		cmdline.add_options()
			("help,h", "produce help message")
			("pid,p", po::value<int>(), "The pid to query. If not specified, "
			 "defaults to the pid of this process.")
			("environ,e", "Also show the environment variables (long output)")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(oall).
			  positional(p).run(), vm);
		po::notify(vm);

		if (vm.count("help"))
		{
			cerr << desc << endl;
			return 1;
		}

		Ryan_Debug::process_static_options(vm);


		bool showEnviron = false;
		if (vm.count("environ")) showEnviron = true;

		int pid = 0;
		if (vm.count("pid")) pid = vm["pid"].as<int>();
		else pid = getPID(); // getPPID(getPID());
		if (!pidExists(pid))
		{
			cerr << "Specified PID " << pid << " is invalid." << endl;
			return 2;
		}

		// Recurse through the pids until a suitable endpoint is found.
		boost::shared_ptr<const processInfo> info;
		//hProcessInfo info = nullptr;
		for (;;)
		{
			if (!pid) break; // Recursed to 0
			info = boost::shared_ptr<const processInfo>(getInfo(pid), freeProcessInfo);
			cout << *(info.get());
			if (showEnviron)
			{
				size_t sEnv = 0;
				const char* cenv = getEnviron(info.get(), sEnv);
				std::string env(cenv,sEnv);
				cout << "Environment Variables:\n" << env << endl;
			}
			cout << endl;
			if (pid == getPPID(info.get())) break; // linux shells
			pid = getPPID(info.get());
			if (!pid) break;
			const char* cname = getName(info.get());
			std::string name(cname);
			if (name == "devenv.exe") break; // VS
			if (name == "screen") break;
			if (name == "bash") break;
			if (name == "tcsh") break;
			if (name == "sshd") break;
			if (name == "cmd.exe") break;
			if (name == "explorer.exe") break;
			if (name == "at") break;
			if (name == "cron") break;
		}

		//ryan_debug::printDebugInfo();
		//ryan_debug::debug_preamble(cerr);
		cout << endl;
	} catch(std::exception & e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}

