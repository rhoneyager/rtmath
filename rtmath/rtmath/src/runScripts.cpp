#include "../rtmath/Stdafx.h"

#include "../rtmath/ddscat/runScripts.h"
#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/ddscat/ddpar.h"

namespace rtmath
{
	namespace ddscat {

		void runScriptIndiv::write(const std::string &spath) const
		{
			using namespace boost::filesystem;
			using namespace std;
			const string dirsuffix = "doRun.csh";

			path p(spath);
			if (boost::filesystem::is_directory(p))
			{
				if (dirsuffix.size())
				{
					p = p / dirsuffix;
				} else {
					throw rtmath::debug::xPathExistsWrongType(spath.c_str());
				}
			}

			boost::filesystem::path tmpgdir, tmpdir, tmppath;

			// Want to calc results in a temp dir. Avoids running ddscat over nfs.....
			tmpgdir = boost::filesystem::temp_directory_path();
			string tmpdirname = "rtmath-ddscat-";
			tmpdirname.append(_uuid);
			tmpdir = boost::filesystem::path(tmpdirname);
			tmppath = tmpgdir / tmpdir;

			ofstream out(p.string().c_str());

			out << "#!/bin/tcsh" << endl;
			out << "## Individual run script for " << _uuid << endl << endl;
			out << "set symlinks=expand" << endl; // allow cd ..
			out << "mkdir " << tmppath << endl;
			out << "ln -s " << tmppath << " run" << endl;
			out << "cp ddscat.par run/" << endl;
			out << "cp diel.tab run/" << endl;

			if (exists(boost::filesystem::path(p/"shape.dat")))
				out << "cp shape.dat run/" << endl;
			out << "cp ddparIterator.xm* run/" << endl;
			out << endl;
			out << "cd run" << endl;
			out << "ddscat\n";
			out << "cd .." << endl;
			
			// TODO: compute stats based on target.out
			if (_gen.shapeStats)
				out << "rtmath-statcalc -i run/ -o run/" << endl;

			out << "tar cjf rtmath-ddscat-indrun-" << _uuid << ".tar.bz2 run/*\n";
			
			// If desired, send to aqua
			if (_gen.doExport && _gen.exportLoc.size())
				out << "cp " << _uuid << ".tar.bz2 " << _gen.exportLoc << endl;

			// Clean up temporary path
			out << "rm -rf " << tmppath << endl;
			out << "rm -rf run" << endl;


		}

		runScriptGlobal::runScriptGlobal(const ddParGenerator &gen)
			: _gen(gen)
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

		void runScriptGlobal::write(const std::string &spath) const
		{
			// Take path. Produce global run script titles doAllRuns.csh
			using namespace boost::filesystem;
			using namespace std;

			const string dirsuffix = "doAllRuns.csh";

			path p(spath);
			if (boost::filesystem::is_directory(p))
			{
				if (dirsuffix.size())
				{
					p = p / dirsuffix;
				} else {
					throw rtmath::debug::xPathExistsWrongType(spath.c_str());
				}
			}

			ofstream out(p.string().c_str());

			out << "#!/bin/tcsh" << endl;
			out << "## Multiple run script " << endl << endl;

			for (auto it = _subdirs.begin(); it != _subdirs.end(); it++)
			{
				out << "echo " << *it << endl;
				out << "tcsh " << *it << "/doRun.csh" << endl;
			}

			out << endl << endl;
		}

	}
}
