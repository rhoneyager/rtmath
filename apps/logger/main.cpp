#include <iostream>
#include <string>
#include <vector>
#include <boost/program_options.hpp>

#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/info.h"

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
		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("pid,p", po::value<int>(), "The pid to query. If not specified, "
			 "defaults to the pid of this process.")
			("environ,e", "Also show the environment variables (long output)")
			;

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(desc).
			  positional(p).run(), vm);
		po::notify(vm);

		if (vm.count("help"))
		{
			cerr << desc << endl;
			return 1;
		}

		bool showEnviron = false;
		if (vm.count("environ")) showEnviron = true;

		int pid = 0;
		if (vm.count("pid")) pid = vm["pid"].as<int>();
		else pid = getPPID(getPID());
		if (!pidExists(pid))
		{
			cerr << "Specified PID " << pid << " is invalid." << endl;
			return 2;
		}

		// Recurse through the pids until a suitable endpoint is found.
		processInfo info;
		for (;;)
		{
			if (!pid) break; // Recursed to 0
			info = getInfo(pid);
			cout << info;
			if (showEnviron)
				cout << "Environment Variables:\n" << info._environ << endl;
			cout << endl;
			if (pid == info.ppid) break; // linux shells
			pid = info.ppid;
			if (info.name == "devenv.exe") break; // VS
			if (info.name == "screen") break;
			if (info.name == "bash") break;
			if (info.name == "tcsh") break;
			if (info.name == "sshd") break;
			if (info.name == "cmd.exe") break;
			if (info.name == "explorer.exe") break;
			if (info.name == "at") break;
			if (info.name == "cron") break;
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

