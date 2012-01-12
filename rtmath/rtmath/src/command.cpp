#include "../rtmath/Stdafx.h"
#include <string>
#include <memory>
#include "../rtmath/rtmath.h"
#include <vector>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cmath>

namespace rtmath {
	namespace config {
		void processCommands(const char* commands)
		{
			// This function processes any received commands.
			// The commands may be terminated by newlines. The command stream 
			// also ends with a null. All commands found will be parsed and executed.

			// Split multiple commands and process each individually:
			using namespace std;
			vector<string> command; // The vector that holds each parsed command.
			// Do this quickly
			{
				vector<string> rawCommand;
				size_t start = 0, end = 0;
				const char csep = '\n';
				size_t endpos = strlen(commands);
				do {
					if (commands[end] == csep || commands[end] == '\0')
					{
						// Process new possible command
						rawCommand.push_back(string(commands[start],commands[end]));
						start = end;
					}
					end++;
				} while (end != endpos);

				// Post-parsing (search for empty commands, remove whitespace, ...)
				for (int i=0;i<(int)rawCommand.size();i++)
				{
					if (rawCommand[i].size() == 0) continue;
					command.push_back(rawCommand[i]);
				}
			}

			// Now to move on to actual command processing:
			// Loop through all commands and execute them.
			// Catch any errors returned and dispatch them to the appropriate listener/
			for (int i=0; i<(int)command.size(); i++)
			{

			}
		}


		// the parseParams class member functions
		parseParams::parseParams(int argc, char** argv)
		{
			_ac = argc;
			_av = argv;
		}

		bool parseParams::readParam(const char* oName, double &val)
		{
			// Find the option
			std::string op(oName);
			for (size_t i=0;i<_ac-1;i++)
			{
				std::string p(_av[i]);
				if (p==op)
				{
					std::string v(_av[i+1]);
					// Convert to double
					val = atof(v.c_str());
					return true;
				}
			}
			return false;
		}

		bool parseParams::readParam(const char* oName, size_t num, double *vals)
		{
			using namespace std;
			string op(oName);
			for (size_t i=0;i<_ac-num;i++)
			{
				string p(_av[i]);
				if (p==op)
				{
					for (size_t j=0;j<num;j++)
					{
						string s(_av[i+1+j]);
						vals[j] = atof(s.c_str());
					}
					return true;
				}
			}
			return false;
		}

		bool parseParams::readParam(const char* oName, std::string &val)
		{
			using namespace std;
			string op(oName);
			for (size_t i=0;i<_ac-1;i++)
			{
				string p(_av[i]);
				if (p==op)
				{
					string r(_av[i+1]);
					val = r;
					return true;
				}
			}
			return false;
		}

		bool parseParams::readParam(const char* oName, bool &flag)
		{
			using namespace std;
			string op(oName);
			for (size_t i=0;i<_ac-1;i++)
			{
				string p(_av[i]);
				if (p==op)
				{
					flag = (bool) atoi(_av[i+1]);
					return true;
				}
			}
			return false;
		}

		bool parseParams::readParam(const char* oName)
		{
			using namespace std;
			string op(oName);
			for (size_t i=0;i<_ac;i++)
			{
				string p(_av[i]);
				if (p==op) return true;
			}
			return false;
		}

	}; // end namespace config
}; // end namespace rtmath


