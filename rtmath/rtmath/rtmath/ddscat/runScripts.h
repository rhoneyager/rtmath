#pragma once
/* Provides individual and mass run scripts for ddParGenerator outputs
 */

#include <string>
#include <set>

namespace rtmath {
	namespace ddscat {

		class ddParGenerator;

		class runScriptIndiv
		{
		public:
			runScriptIndiv(const std::string &uuid, const ddParGenerator &gen)
				: _uuid(uuid), _gen(gen) {}
			void write(const std::string &path) const;
		private:
			std::string _uuid;
			const ddParGenerator &_gen;
		};

		class runScriptGlobal
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


