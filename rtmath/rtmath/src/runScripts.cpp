#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <boost/filesystem.hpp>
#include "../rtmath/ddscat/runScripts.h"

namespace rtmath
{
	namespace ddscat {
		std::string runScriptIndiv::_exportLoc;
		bool runScriptIndiv::_doStats = false;

		void runScriptIndiv::write(const std::string &path) const
		{
			using namespace boost::filesystem;
			using namespace std;
			boost::filesystem::path p(path);
			boost::filesystem::path base, baseabs, script, tmpgdir, tmpdir, tmppath;
			if (is_directory(p)) base = p;
			else base = p.parent_path();
			boost::filesystem::absolute(baseabs, base);
			script = base / "doRun.csh";

			// Want to calc results in a temp dir. Avoids running ddscat over nfs.....
			tmpgdir = boost::filesystem::temp_directory_path();
			string tmpdirname = "rtmath-ddscat-";
			tmpdirname.append(_uuid);
			tmpdir = boost::filesystem::path(tmpdirname);
			tmppath = tmpgdir / tmpdir;

			ofstream out(script.string().c_str());

			out << "#!/bin/tcsh" << endl;
			out << "## Individual run script for " << _uuid << endl << endl;
			out << "mkdir " << tmppath << endl;
			out << "ln -s " << tmppath << " run" << endl;
			out << "cp ddscat.par " << tmppath << endl;
			out << "cp diel.tab " << tmppath << endl;

			if (exists(boost::filesystem::path(base/"shape.dat")))
				out << "cp shape.dat " << tmppath << endl;
			if (exists(boost::filesystem::path(base/"RUNDEFS.txt")))
				out << "cp RUNDEFS.txt " << tmppath << endl;
			out << endl;
			out << "cd " << tmppath << endl;
			out << "ddscat\n";
			out << "cd " << baseabs << endl;

			// TODO: compute stats based on target.out
			if (_doStats)
				out << "rtmath-statcalc -i " << tmppath << " -o " << tmppath << endl;

			out << "tar cjf rtmath-ddscat-indrun-" << _uuid << ".tar.bz2 " << tmppath << "/*\n";

			// If desired, send to aqua
			if (_exportLoc.size())
				out << "cp " << _uuid << ".tar.bz2 " << _exportLoc << endl;

			// Clean up temporary path
			out << "rm -rf " << tmppath << endl;
			out << "rm -rf run" << endl;


		}

		runScriptGlobal::runScriptGlobal()
		{
		}

		void runScriptGlobal::addSubdir(const std::string &dirname)
		{
			_subdirs.insert(dirname);
		}

		void runScriptGlobal::addSubdir(const std::set<std::string> &dirname)
		{
			for (auto it = dirname.begin(); it != dirname.end(); it++)
				_subdirs.insert(*it);
		}

		void runScriptGlobal::write(const std::string &path) const
		{
			// Take path. Produce global run script titles doAllRuns.csh
			using namespace boost::filesystem;
			using namespace std;
			boost::filesystem::path p(path);
			boost::filesystem::path base, baseabs, script, tmpgdir, tmpdir, tmppath;
			if (is_directory(p)) base = p;
			else base = p.parent_path();
			boost::filesystem::absolute(baseabs, base);
			script = base / "doAllRuns.csh";

			ofstream out(script.string().c_str());

			out << "#!/bin/tcsh" << endl;
			out << "## Multiple run script " << endl << endl;

			for (auto it = _subdirs.begin(); it != _subdirs.end(); it++)
				out << "tcsh " << *it << "/doRun.csh" << endl;

			out << endl << endl;
		}

	}
}
