//#include "frmmain.h"
#include "converter.h"
//#include <QtGui/QApplication>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"

int main(int argc, char *argv[])
{
	using namespace std;
	try {
		/*
		if (argc == 1)
		{
			QApplication a(argc,argv);
			frmMain w;
			w.show();
			return a.exec();
		}
		*/
		// Otherwise, parse options
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("force-180", "Have tmatrix converter only compute phase matrices for backscatter "
			 "angle. Used to speed up conversion.")
			("flip-S", "When computing backscatter, provide a Mueller matrix produced by flipping "
			 "the real and imaginary components of the Jones matrix. Use this option to match for "
			 "exponent sign conventions between T-matrix and DDSCAT.")

			("temperature,T", po::value<double>()->default_value(267), 
			 "Specify temperature (K). Not necessary / overridden if diel.tab is specified.")
			("frequency,F", po::value<double>()->default_value(0),
			 "Override frequency (GHz)")
			("dipole-spacing,d", po::value<double>()->default_value(0),
			 "Override dipole spacing to a fixed value (um)")
			("dipole-spacing-file", po::value<string>(),
			 "Get dipole spacings from the specified file. Useful in Liu conversions.")
			("disable-qhull", "Disable qhull calculations for the shapes. Needed for Liu dendrites.")
			("suffix", po::value<string>(), "Append suffix to generated files")
			("nu,n", po::value<double>()->default_value(0.85), 
			 "Specify nu for Sihvola")

			("output-dir", po::value<string>(), "Override output directory. If multiple files in the input have the same name, this will corrupt the output. When not specified, results are saved in the same directory as the inputs.")
			("default-par,p", po::value<string>(), 
			 "Specify default ddscat.par file")
			("diel-tab", po::value<string>(), 
			 "Specify diel.tab file. With this, the temperature is no longer needed.")
			("shapefiles,s", po::value<vector<string> >(), 
			 "Specify shapefiles or shape statistics files. Files are separated based on extension.")
			("shape-method", po::value<string>()->default_value("Same RMS aspect ratio"), 
			 "Specify shape method (Same RMS aspect ratio/samerms, Same real aspect ratio/sameabs, Equiv Aeff Sphere/equivaeff)")
			("diel-method", po::value<string>()->default_value("Sihvola"), 
			 "Specify dielectric method (Sihvola, Debye, Maxwell-Garnett)")
			("volfrac-method", po::value<string>()->default_value("minsphere"), 
			 "Specify volume fraction method (Minimal circumscribing sphere/minsphere, Convex hull/convex, Max Ellipsoid/ellipmax, RMS Ellipsoid/elliprms)")
			("use-mie", "Perform Mie runs instead of T-matrix")
			("mie-order", po::value<int>()->default_value(0), "Approximate Mie scattering with ith series. Use 1 for Rayleigh, ..., 0 for full Mie.")
			;

		po::positional_options_description p;
		p.add("shapefiles",-1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		bool qhull_enabled = true;
		bool force180 = false;
		bool flipS = false;
		bool useMie = false;
		int mieOrder = vm["mie-order"].as<int>();

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 2;
		}

		if (vm.count("force-180"))
			force180 = true;

		if (vm.count("flip-S"))
			flipS = true;

		if (vm.count("use-mie"))
			useMie = true;

		if (vm.count("disable-qhull")) qhull_enabled = false;
		rtmath::ddscat::shapeFileStats::doQhull(qhull_enabled);

		vector<string> inputs;
		if (vm.count("shapefiles"))
		{
			inputs = vm["shapefiles"].as< vector<string> >();

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
			cerr << "Default ddscat.par: " << defaultPar << std::endl;
		} else {
			cerr << "Specifying default ddscat.par file is highly recommended." << endl;
		}
		string dieltab;
		double T = vm["temperature"].as<double>();
		if (vm.count("diel-tab"))
		{
			dieltab = vm["diel-tab"].as<string>();
			cerr << "Using diel file: " << dieltab << endl;
		} else {
			
			cerr << "Temperature: " << T << " K\n";
		}
		double nu = vm["nu"].as<double>();
		cerr << "Nu: " << nu << endl;
		double freq = vm["frequency"].as<double>();
		cerr << "Frequency: " << freq << " GHz\n";
		double dipoleSpacing = vm["dipole-spacing"].as<double>();
		string dipoleFile;
		if (vm.count("dipole-spacing-file")) dipoleFile = vm["dipole-spacing-file"].as<string>();
		if (dipoleFile.size())
			cerr << "Extracting dipole spacings from: " << dipoleFile << endl;
		if (dipoleSpacing)
			cerr << "Dipole spacing defaults to: " << dipoleSpacing << " um." << endl;
		else
			cerr << "\tAUTODETECTING DIPOLE SPACING\n";

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
		cerr << sm << endl << dm << endl << vmeth << endl;

		if (vm.count("suffix"))
		{
			suffix = vm["suffix"].as<string>();
		} else {
			// Create an intelligent suffix describing the run
			ostringstream s;
			s << "-d-" << dipoleSpacing << "-nu-" << nu 
				<< "-F-" << freq << "-dm-" << dm;
			// Volume fraction and shape method need conversion to short form
			s << "-vm-";
			if (vmeth == "Minimal circumscribing sphere") s << "minsphere";
			if (vmeth == "Convex hull") s << "convex";
			if (vmeth == "Max Ellipsoid") s << "ellipmax";
			if (vmeth == "RMS Ellipsoid") s << "elliprms";
			s << "-sm-";
			if (sm == "Same RMS aspect ratio") s << "samerms";
			if (sm == "Same real aspect ratio") s << "sameabs";
			if (sm == "Equiv Aeff Sphere") s << "equivaeff";

			suffix = s.str();
		}
		
		// If a dipole spacing file has been provided, then read it
		map<string, double> dipoleMap;
		if (dipoleFile.size())
		{
			cerr << "Reading dipole map\n";
			using namespace boost::filesystem;
			path p(dipoleFile);
			if (!exists(p)) throw rtmath::debug::xMissingFile(dipoleFile.c_str());
			ifstream din(dipoleFile.c_str());
			// Skip the header line
			string lin;
			std::getline(din,lin);
			// Read each line, and extract the first two columns (separated by tabs)
			while (din.good())
			{
				std::getline(din,lin);
				istringstream is(lin);
				string key;
				double ds;
				is >> key >> ds;
				if (dipoleMap.count(key) == 0)
				{
					dipoleMap[key] = ds;
					//cerr << "\t" << key << "\t" << ds;
				}

			}
		}

		// Iterate through files, try to find appropriate ddscat.par, and perform conversion
		for (auto it = inputs.begin(); it != inputs.end(); ++it)
		{
			std::cerr << "Converting " << *it << std::endl;
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
			
			string pDest = *it; // Here because I will strip some endings on some files

			boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;
			// Look at file name. If it has .stats.xml, then it is a precomputed stats file.
			if (it->find(".stats.xml") != string::npos)
			{
				stats = boost::shared_ptr<rtmath::ddscat::shapeFileStats>(new rtmath::ddscat::shapeFileStats);
				rtmath::serialization::read<rtmath::ddscat::shapeFileStats>
					(*stats,*it);
				auto i = it->find(".stats.xml");
				pDest = pDest.substr(0,i); // Strip exerything after and including .stats.xml
			} else {
				stats = rtmath::ddscat::shapeFileStats::genStats(*it);
			}
			string dipoleKey = path(pDest).filename().string();
			cerr << "Dipole key is: " << dipoleKey << endl;

			if (vm.count("output-dir"))
			{
				using namespace boost::filesystem;
				path pD(pDest);
				path pbase = pD.parent_path();
				path fn = pD.filename();
				string so = vm["output-dir"].as<string>();
				path pso(so);
				pso.remove_filename();

				if (!exists(pso)) throw rtmath::debug::xMissingFile(pso.string().c_str());
				path peff = pso / fn;
				pDest = peff.string();
			}
			if (suffix.size())
			{
				pDest.append("-");
				pDest.append(suffix);
			}
			pDest.append("-tmatrix.xml");

			fileconverter cnv;
			cnv.setFlipS(flipS);
			cnv.setStats(stats);
			cnv.setDDPARfile(pCand.string());
			cnv.setShapeMethod(sm);
			cnv.setDielMethod(dm,nu);
			cnv.setVolFracMethod(vmeth);
			cnv.setDielFile(dieltab);
			cnv.setTemp(T);
			cnv.setMie(useMie,mieOrder);
			cnv.setFreq(freq);
			if (dipoleMap.count(dipoleKey))
			{
				cerr << "Overriding dipole spacing: " << dipoleMap.at(dipoleKey) << endl;
				cnv.setDipoleSpacing(dipoleMap.at(dipoleKey));
			}
			else
			{
				cerr << "Using default dipole spacing " << dipoleSpacing << endl;
				cnv.setDipoleSpacing(dipoleSpacing);
			}
			cnv.setForce180(force180);

			cnv.convert(pDest);
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
