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

		class runScriptIndiv
		{
		public:
			runScriptIndiv(const std::string &uuid)
				: _uuid(uuid) {}
			void write(const std::string &path) const;
			static void exportLoc(const std::string &loc) { _exportLoc = loc; }
			static void doStats(bool doit) {_doStats = doit; }
		private:
			std::string _uuid;
			static std::string _exportLoc;
			static bool _doStats;
		};

	}
}


