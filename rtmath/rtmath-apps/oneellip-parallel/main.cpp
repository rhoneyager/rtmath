/** This program can execute T-matrix requests in parallel, over a network. It 
* has a few main components. First, the command line is parsed, establishing the 
* type of run. If run in multi-processor mode, then the process spawns multiple 
* copies of itself, which communicate using shared memory.
*
* The master process then either listens for network connections or executes pre-defined 
* commands (from the command line or from a xml file). If executing from a command line / 
* config file, the process terminates after processing is done. If waiting on a network 
* connection, it will wait for a subsequent connection. Child processes only 
* last as long as their parent does.
**/

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <fstream>
#include <complex>
#include <set>
#include <vector>
#include <sstream>
#include <string>
#include <QObject>
#include <QTimer>
#include <QApplication>
#include <QProcess>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "app.h"
#include "comm.h"
#include "dispatcher.h"
#include "worker.h"
#include "main.h"

/*
/// Useful function to get public program-wide objects (like the logger and communicator)
appOneellipServer* appOneellipServer::instance()
{
	static appOneellipServer *m = nullptr;
	if (!m) m = new appOneellipServer;
	return m;
}
*/

namespace po = boost::program_options;
void addRunOptions(po::options_description &cmdline, 
	po::options_description &config, po::options_description &hidden,
	po::options_description &runmatch, po::options_description &refract,
	po::options_description &basic, po::options_description &scale)
{
	using namespace std;
	rtmath::debug::add_options(cmdline, config, hidden);
	rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
	//rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);
	//rtmath::refract::add_options(cmdline, config, hidden);

	runmatch.add_options()
		("dipole-spacing,d", po::value<double>(), "Set dipole spacing for file exports.")
		("input-shape,i", po::value< vector<string> >(), "Input shape files")
		("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
		("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
		("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
		("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
		("match-dipole-spacing", po::value<vector<float> >()->multitoken(), "Match typical dipole spacings")
		("match-dipole-numbers", po::value<vector<size_t> >()->multitoken(), "Match typical dipole numbers")
		("match-parent-flake-hash", po::value<vector<string> >()->multitoken(), "Match flakes having a given parent hash")
		("match-parent-flake", "Select the parent flakes")

		("vf-scaling", po::value<string>()->default_value("Circumscribing_Sphere"),
		"Select the method used in determining the volume fraction. 1) Voronoi_Internal uses the internal volume fraction."
		" 2) Circumscribing_Sphere, 3) Convex, 4) Ellipsoid_Max uses the max circumscribing ellipsoid,"
		" 5) RMS_Sphere for a root mean square sphere (per Petty and Huang 2010),"
		" 6) Gyration_Sphere uses the Westbrok 2006 radius of gyration,"
		" 7) Solid_Sphere is an optically hard sphere.")
		("voronoi-depth", po::value<size_t>()->default_value(2), "Sets the internal voronoi depth for scaling")

		("ar-method", po::value<string>()->default_value("Max_Ellipsoids"),
		"Max_Ellipsoids: Force aspect ratios to be ellipsoids, following the max AR calculated in stats code. "
		"Spheres: Force aspect ratios to be spheres, instead of stats-determined spheroids. "
		"(TODO SA_V_Ellipsoids: Select AR of ellipsoid with matching surface area to volume ratio).")
		;
	refract.add_options()
		("refract-method", po::value<string>()->default_value("Maxwell_Garnett_Ellipsoids"), "Method used to calculate the resulting dielectric "
		"(Sihvola, Debye, Maxwell_Garnett_Spheres, Maxwell_Garnett_Ellipsoids, Bruggeman). "
		"Only matters if volume fractions are given. Then, default is Maxwell_Garnett_Ellipsoids.")
		("temps,T", po::value<std::string>()->default_value("263"), "Specify temperatures in K")
		("mr", po::value<double>(), "Override real refractive index value")
		("mi", po::value<double>(), "Override imaginary refractive index value")
		("volume-fractions,v", po::value<std::string>()->default_value("1"), "Set the ice volume fractions. Used when not doing shapefiles.")
		("nu,n", po::value<double>()->default_value(0.85), "Value of nu for Sihvola refractive index scaling")
		;
	basic.add_options()
		("aspect-ratios,s", po::value<std::string>()->default_value("1"), "Specify aspect ratio for ellipsoids.")
		("aeffs,a", po::value<std::string>(), "Specify the effective radii in um")
		("radii,r", po::value<std::string>(), "Specify the actual mean sphere radii in um")
		("freqs,f", po::value<std::string>(), "Specify frequencies in GHz. Needed for dielectrics.")
		;
	scale.add_options()
		("scale-aeff", po::value<bool>()->default_value(true), "Scale effective radius based on volume fraction")
		("scale-m", po::value<bool>()->default_value(true), "Scale refractive index based on volume fraction")
		//("scale-v", "Scale for an equivalent volume ellipsoid")
		//("scale-sa", "Scale for an equivalent surface area ellipsoid")
		;
	cmdline.add_options()
		("help,h", "produce help message")
		("output-prefix,o", po::value<string>()->default_value("output"), "Set output file (required)")
		//("alphas", po::value<string>()->default_value("0"), "Set first rotation for backscatter calculation. UNDER DEVELOPMENT.")
		//("betas", po::value<string>()->default_value("0"), "Set second rotation for backscatter calculation. UNDER DEVELOPMENT.")
		//("random-rotations,r", po::value<size_t>(),
		//"Replaces standard alpha and beta angles with random points.")
		("client-id", po::value<int>(), "set client id")
		("server", "App is a server")
		("local-only", "Run server without ICE.")
		("num-cpus,c", po::value<size_t>()->default_value
		(rtmath::debug::getConcurrentThreadsSupported()), "set number of workers")
		("input-file,i", po::value< vector<string> >(), "input file")
		("output-file,o", po::value<string>(), "Set output file (required)")
		;
}

int main(int argc, char *argv[])
{
	using namespace std;
	using std::cout;
	using std::cerr;
	try {
		cerr << "rtmath-oneellip-parallel" << endl;
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			runmatch("Run-matching options"), basic("Basic ellipsoid options"),
			refract("Refractive index options"), scale("Scaling options"),
			config("Config options"), hidden("Hidden options"), oall("all options");
		
		addRunOptions(cmdline, config, hidden, runmatch, refract, basic, scale);
		
		desc.add(cmdline).add(config).add(runmatch).add(refract).add(basic).add(scale);
		oall.add(cmdline).add(runmatch).add(refract).add(config).add(hidden).add(basic).add(scale);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		//rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);
		//rtmath::ddscat::ddOutput::process_static_options(vm);
		//rtmath::refract::process_static_options(vm);

		// Begin checking parameters
		auto doHelp = [&](const std::string& s)
		{
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() == 0) doHelp("");

		/*
		// Set the 'BuildId' property displayed in the IceGridAdmin GUI
		Ice::InitializationData initData;
		initData.properties = Ice::createProperties(argc, argv);
		initData.properties->setProperty("BuildId", string("Ice ") + ICE_STRING_VERSION);

		// Load config.dispatcher by default
		if (argc == 1)
			//if (boost::filesystem::exists("config.dispatcher"))
			initData.properties->load("config.dispatcher"); // A nice default
		//else { std::cerr << "Missing config file" << std::endl; throw; }
		appMaproom* a = appMaproom::instance();

		/// Construct communicator
		Ice::CommunicatorPtr communicator = Ice::initialize(argc, argv, initData);
		rtmath::apps::oneellipParallel::defaultConnunicator = communicator;
		Ice::StringSeq args = Ice::argsToStringSeq(argc, argv);

		a->start("dispatcher", communicator, args);

		communicator->waitForShutdown();
		a->stop();
		*/

		QCoreApplication a(argc, argv);

		string appexec(argv[0]);

		int ppid = 0;
		int myid = 0;
		if (vm.count("client-id"))
		{
			Ryan_Debug::waitOnExit(false);
			myid = vm["client-id"].as<int>();
			int pid = Ryan_Debug::getPID();
			ppid = Ryan_Debug::getPPID(pid);
			cout << "Subprocess " << myid << " has pid " << pid << " with ppid " << ppid << endl;
			
			rtmath::apps::oneellipParallel::Worker worker;
			worker.setParent(ppid, myid);
			QObject::connect(&worker, SIGNAL(terminated()), &a,
				SLOT(quit()), Qt::QueuedConnection);
			QObject::connect(&worker, SIGNAL(attached()), &worker,
				SLOT(startLoop()), Qt::QueuedConnection);
			QObject::connect(&a, SIGNAL(aboutToQuit()), &worker,
				SLOT(close()), Qt::DirectConnection);
			QTimer::singleShot(0, &worker, SLOT(load()));
			return a.exec();
		} else {
			auto app = rtmath::apps::oneellipParallel::appOneellip::instance(argc, argv, vm);
			QTimer::singleShot(0, app, SLOT(startupBegin()));
			return a.exec();
		}

	}
	catch (std::exception &e)
	{
		cerr << "Exception: " << e.what() << endl;
		exit(2);
	}
	catch (...)
	{
		cerr << "Caught unidentified error... Terminating." << endl;
		exit(1);
	}
}


