#include "Stdafx-core.h"
#include <string>
#include <vector>
#include <set>
#include <boost/tokenizer.hpp>
#include <boost/filesystem.hpp>
#include "../rtmath/command.h"

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
			_flag = '-';
		}

		bool parseParams::readParam(const char* oName)
		{
			using namespace std;
			string op(oName);
			for (size_t i=0;i<(size_t) (_ac);i++)
			{
				string p(_av[i]);
				if (p==op) return true;
			}
			return false;
		}

		findFile::findFile()
		{
		}

		findFile::findFile(const std::string &searchPath, 
			const std::string &suffixes)
		{
			addSearch(searchPath);
			addSuffix(suffixes);
		}

		findFile::findFile(const std::vector<std::string> &searchPath, 
			const std::set<std::string> &suffixes)
		{
			setSearch(searchPath);
			setSuffix(suffixes);
		}

		void findFile::setSearch(const std::vector<std::string> &searchPath)
		{
			_paths = searchPath;
		}

		void findFile::clearSearch()
		{
			_paths.clear();
		}

		void findFile::addSearch(const std::string &searchPath)
		{
			// Time to tokenize
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",");
			tokenizer tokens(searchPath, sep);
			for (tokenizer::iterator it = tokens.begin();
				it != tokens.end(); ++it)
			{
				if (it->size())
					_paths.push_back(*it);
			}
		}

		void findFile::addSearch(const boost::filesystem::path &searchPath)
		{
			// Assumes that just a single path exists
			if (searchPath.string().size())
				addSearch(searchPath.string());
		}

		void findFile::clearSuffix()
		{
			_suffixes.clear();
		}

		void findFile::setSuffix(const std::set<std::string> &suffixes)
		{
			_suffixes = suffixes;
		}

		void findFile::addSuffix(const std::string &suffix)
		{
			// Tokenize, svp.
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",");
			tokenizer tokens(suffix, sep);
			for (tokenizer::iterator it = tokens.begin();
				it != tokens.end(); ++it)
			{
				// Check for existence (while allowing for empty suffix)
				if (_suffixes.count(*it) == 0)
					_suffixes.insert(*it);
			}
			// Add empty suffix
			if (_suffixes.count("") == 0) _suffixes.insert("");
		}

		bool findFile::searchSubDir() const
		{
			return false;
		}

		bool findFile::search(const std::string &token, std::string &res) const
		{
			res.clear();

			// The function that actually does searches based on the provided paths
			// Iterate through each path
			std::vector<std::string>::const_iterator it;
			for (it=_paths.begin();it!=_paths.end();it++)
			{
				// Iterate through all possible suffixes
				std::set<std::string>::const_iterator ot;
				for (ot=_suffixes.begin();ot!=_suffixes.end();ot++)
				{
					// If an empty suffix is missing, then this will NOT match
					// against just the raw token
					using namespace boost::filesystem;
					using namespace std;
					string filename = token;
					filename.append(*ot); // Append the current suffix
					path cand = path(*it) / path(filename);
					// Return both files and directories...
					if (exists(cand))
						//if (!is_directory(cand)) 
						{
							// Success
							res = cand.string();
							return true;
						}
				}
			}

			// If we hit this point, then the file was not found
			return false;
		}

	}
}


