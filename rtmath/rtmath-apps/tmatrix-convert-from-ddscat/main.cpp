#include "frmmain.h"
#include "converter.h"
#include <QtGui/QApplication>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"

int main(int argc, char *argv[])
{
	using namespace std;
	try {
		if (argc == 1)
		{
			QApplication a(argc,argv);
			frmMain w;
			w.show();
			return a.exec();
		}

		// Otherwise, parse options
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("ROOT-output,r", "produce ROOT conversion output (for plotting and debugging)")
			("flip-tmatrix", "Perform S real / imaginary swap to correct tmatrix run results")
			("temperature,T", po::value<double>()->default_value(267), 
			 "Specify temperature (K)")
			("frequency,F", po::value<double>()->default_value(0),
			 "Override frequency (GHz)")
			("dipole-spacing,d", po::value<double>()->default_value(0),
			 "Override dipole spacing (um)")
			("suffix", po::value<string>(), "Append suffix to generated files")
			("nu,n", po::value<double>()->default_value(0.85), 
			 "Specify nu for Sihvola")

			("default-par,p", po::value<string>(), 
			 "Specify default ddscat.par file")
			("shapefiles,s", po::value<vector<string> >(), 
			 "Specify shapefiles")
			("shape-method", po::value<string>()->default_value("Same RMS aspect ratio"), 
			 "Specify shape method (Same RMS aspect ratio/samerms, Same real aspect ratio/sameabs, Equiv Aeff Sphere/equivaeff)")
			("diel-method", po::value<string>()->default_value("Sihvola"), 
			 "Specify dielectric method (Sihvola, Debye, Maxwell-Garnett)")
			("volfrac-method", po::value<string>()->default_value("Convex hull"), 
			 "Specify volume fraction method (Minimal circumscribing sphere/minsphere, Convex hull/convex, Max Ellipsoid/ellipmax, RMS Ellipsoid/elliprms)");

		po::positional_options_description p;
		p.add("shapefiles",-1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		bool ROOT = false;
		bool fliptm = false;

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 2;
		}

		if (vm.count("ROOT-output"))
			ROOT = true;

		if (vm.count("flip-tmatrix"))
			fliptm = true;

		vector<string> inputs = vm["shapefiles"].as< vector<string> >();
		if (vm.count("shapefiles"))
		{
			cerr << "Input shapefiles are:" << endl;
			for (auto it = inputs.begin(); it != inputs.end(); it++)
				cerr << "\t" << *it << "\n";
		} else {
			cerr << "Need to specify input shapefiles.\n" << desc << endl;
			return 3;
		}

		using namespace boost::filesystem;
		string defaultPar;
		if (vm.count("default-par"))
		{
			defaultPar = vm["default-par"].as<string>();
			path p(defaultPar);
			if (!exists(p))
				throw rtmath::debug::xMissingFile(defaultPar.c_str());
		} else {
			cerr << "Specifying default ddscat.par file is highly recommended." << endl;
		}
		double T = vm["temperature"].as<double>();
		double nu = vm["nu"].as<double>();
		double freq = vm["frequency"].as<double>();
		double dipoleSpacing = vm["dipole-spacing"].as<double>();

		string sm, dm, vmeth, suffix;
		if (vm.count("shape-method"))
			sm = vm["shape-method"].as<string>();
		{
			if (sm == "samerms")
				sm = "Same RMS aspect ratio";
			if (sm == "sameabs")
				sm = "Same real aspect ratio";
			if (sm == "equivaeff")
				sm = "Equiv Aeff Sphere";
		}
		if (vm.count("diel-method"))
			dm = vm["diel-method"].as<string>();
		if (vm.count("volfrac-method"))
			vmeth = vm["volfrac-method"].as<string>();
		{
			if (vmeth == "minsphere")
				vmeth = "Minimal circumscribing sphere";
			if (vmeth == "convex")
				vmeth = "Convex hull";
			if (vmeth == "ellipmax")
				vmeth = "Max Ellipsoid";
			if (vmeth == "elliprms")
				vmeth = "RMS Ellipsoid";
		}

		if (vm.count("suffix"))
			suffix = vm["suffix"].as<string>();
		

		// Iterate through files, try to find appropriate ddscat.par, and perform conversion
		for (auto it = inputs.begin(); it != inputs.end(); ++it)
		{
			path p(*it);
			if (!exists(p))
				throw rtmath::debug::xMissingFile(it->c_str());
			path pCand = p.parent_path() / "ddscat.par";
			path pBase = p.parent_path();

			if (!exists(pCand)) 
			{
				if (!exists(path(defaultPar)))
					throw rtmath::debug::xMissingFile(pCand.string().c_str());
				pCand = path(defaultPar);
			}

			boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;
			stats = rtmath::ddscat::shapeFileStats::genStats(*it);

			string pDest = p.string();
			pDest.append("-tmatrix");
			if (suffix.size())
			{
				pDest.append("-");
				pDest.append(suffix);
			}
			pDest.append(".xml");

			fileconverter cnv;
			cnv.setStats(stats);
			cnv.setDDPARfile(pCand.string());
			cnv.setShapeMethod(sm);
			cnv.setDielMethod(dm,nu);
			cnv.setVolFracMethod(vmeth);
			cnv.setTemp(T);
			cnv.setFreq(freq);
			cnv.setDipoleSpacing(dipoleSpacing);
			cnv.setFixTM(fliptm);

			cnv.convert(pDest, ROOT);
		}

		return 0;
	}
	catch (std::exception &e)
	{
		cerr << "Exception caught\n";
		cerr << e.what() << endl;
		return 1;
	}
}
