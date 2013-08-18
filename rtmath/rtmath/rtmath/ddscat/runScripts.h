#pragma once
#include "../defs.h"
/* Provides individual and mass run scripts for ddParGenerator outputs
 */

#include <string>
#include <set>

namespace rtmath {
	namespace ddscat {

		class ddParGenerator;

		class DLEXPORT_rtmath_ddscat runScriptIndiv
		{
		public:
			runScriptIndiv(const std::string &uuid, const ddParGenerator &gen)
				: _uuid(uuid), _gen(gen) {}
			// Write the script
			void write(const std::string &path) const;
			// Add a file that gets copied into the run directory
			void addFile(const std::string &path);
			void setRunCmds(const std::string &cmd);
		private:
			std::string _uuid;
			std::string _runcmd;
			const ddParGenerator &_gen;
			std::set<std::string> _files;
		};

		class DLEXPORT_rtmath_ddscat runScriptGlobal
		{
		public:
			runScriptGlobal(const ddParGenerator &gen);
			void addSubdir(const std::string &dirname);
			void addSubdir(const std::set<std::string> &dirname);
			void write(const std::string &path) const;
		private:
			std::set<std::string> _subdirs;
			const ddParGenerator &_gen;
		};

	}
}


