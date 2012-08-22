#include "frmmain.h"
#include "converter.h"
#include <QtGui/QApplication>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"

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
			("temperature,T", po::value<double>()->default_value(267), 
			 "Specify temperature (K)")
			("nu,n", po::value<double>()->default_value(0.85), 
			 "Specify nu for Sihvola")

			("default-par,p", po::value<string>(), 
			 "Specify default ddscat.par file")
			("shapefiles,s", po::value<vector<string> >(), 
			 "Specify shapefiles")
			("shape-method", po::value<string>()->default_value("Same RMS aspect ratio"), 
			 "Specify shape method (Same RMS aspect ratio, Same real aspect ratio, Equiv Aeff Sphere)")
			("diel-method", po::value<string>()->default_value("Sihvola"), 
			 "Specify dielectric method (Sihvola, Debye, Maxwell-Garnett)")
			("volfrac-method", po::value<string>()->default_value("RMS Ellipsoid"), 
			 "Specify volume fraction method (Minimal circumscribing sphere, Convex hull, Max Ellipsoid, RMS Ellipsoid)");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).run(), vm);
		po::notify(vm);    

		bool ROOT = false;

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 2;
		}

		if (vm.count("ROOT-output"))
			ROOT = true;

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

		string sm, dm, vmeth;
		if (vm.count("shape-method"))
			sm = vm["shape-method"].as<string>();
		if (vm.count("diel-method"))
			dm = vm["diel-method"].as<string>();
		if (vm.count("volfrac-method"))
			vmeth = vm["volfrac-method"].as<string>();

		

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
			pDest.append("-tmatrix.xml");

			fileconverter cnv;
			cnv.setStats(stats);
			cnv.setDDPARfile(pCand.string());
			cnv.setShapeMethod(sm);
			cnv.setDielMethod(dm,nu);
			cnv.setVolFracMethod(vmeth);
			cnv.setTemp(T);

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
