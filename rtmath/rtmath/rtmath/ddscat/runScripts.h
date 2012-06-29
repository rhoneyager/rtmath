#pragma once
/* Provides individual and mass run scripts for ddParGenerator outputs
 */

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <complex>
#include <boost/tokenizer.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include "ddpar.h"

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


