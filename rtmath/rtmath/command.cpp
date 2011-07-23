#include "Stdafx.h"
#include "command.h"
#include <string>
#include <memory>
#include "rtmath.h"
#include <vector>
#include <iostream>
#include <cstdlib>
#include <cstring>

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
				for (int i=0;i<rawCommand.size();i++)
				{
					if (rawCommand[i].size() == 0) continue;
					command.push_back(rawCommand[i]);
				}
			}

			// Now to move on to actual command processing:
			// Loop through all commands and execute them.
			// Catch any errors returned and dispatch them to the appropriate listener/
			for (int i=0; i<command.size(); i++)
			{

			}
		}
	}; // end namespace config
}; // end namespace rtmath


