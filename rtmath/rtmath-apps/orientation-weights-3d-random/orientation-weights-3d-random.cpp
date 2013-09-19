/* orientation-weights-3d-random
 * This program generates random rotation angles for ddscat testing.
 */
#pragma warning( push )
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <algorithm>
#include <cmath>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random/random_device.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#pragma warning( pop ) 

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	const double pi = boost::math::constants::pi<double>();

	try {
		cerr << "rtmath-orientation-weights-3d-random\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);
		//p.add("output", 2);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("numpoints,n", po::value<size_t>()->default_value(6156),
			"Specify the number of random points to generate")
			("input,i", po::value<string>(), "input skeleton ddscat directory. "
			"If specified, this program will create a separate ddscat run per "
			"random orientation selected.")
			("output,o", po::value<string>(), "Base name for the outputs. "
			"This includes the .tsv file that lists the random points, "
			"and if run generation is enabled: an output .csh script to perform "
			"and summarize the runs and a directory tree containing the outputs.")
			("modules", po::value<string>()->
				default_value("intel ddscat/7.3.0_130527_r1-intel-openmp-checked"), 
				"List modulefiles to load (as a quoted string).")
			//("betas,b", po::value<string >(), "Specify beta rotation range")
			//("thetas,t", po::value<string > (), "Specify theta rotation range")
			//("phis,p", po::value<string >(), "Specify phi rotation range")
			//("mean_theta", po::value<double>()->default_value(0),
			//"Theta mean (degrees)")
			//("mean_phi", po::value<double>()->default_value(0),
			//"Phi mean (degrees)")
			//("kappa", po::value<double>()->default_value(10),
			//"Kappa (degrees) is comparable to 1/sigma^2. Limit of infinity for uniform distribution, "
			//"and 0 for only the mean.")
			//("method", po::value<string>()->default_value("uniform"), "Specify the method used in the orientation "
			//"calculations (uniform, vMF, vMFdual).");
			;

		hidden.add_options()
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		if (!vm.count("output")) doHelp("Must specify an output file.");

		using namespace boost::filesystem;
		bool createRun = false;
		boost::shared_ptr<ddscat::ddPar> parfile;
		path pSkel;
		vector<path> vSkels;
		auto expandSymlinks = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (is_symlink(p))
			{
				path pf = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				return pf;
			} else {
				return p;
			}
		};
		if (vm.count("input"))
		{
			createRun = true;
			pSkel = path(vm["input"].as<string>());
			path pPar = pSkel / "ddscat.par";
			if (!exists(pPar)) throw rtmath::debug::xMissingFile(pPar.string().c_str());
			parfile = boost::shared_ptr<ddscat::ddPar>(new ddscat::ddPar(pPar.string()));
			{
				path ps = expandSymlinks(pSkel);
				if (is_directory(ps))
				{
					copy(directory_iterator(ps), 
						directory_iterator(), back_inserter(vSkels));
				} else throw rtmath::debug::xPathExistsWrongType(pSkel.string().c_str());
			}
		}

		//Ryan_Serialization::process_static_options(vm);
		//ddscat::shapeFileStats::process_static_options(vm);

		typedef boost::random::mt19937 gen_type;
		gen_type rand_gen;
		rand_gen.seed(static_cast<unsigned int>(std::time(0)));
		boost::random::uniform_on_sphere<float> dist(3);
		boost::variate_generator<gen_type&, boost::uniform_on_sphere<float> >
			random_on_sphere(rand_gen, dist);
		const float pi = boost::math::constants::pi<float>();
		boost::random::uniform_real_distribution<float> distUni(0,2.f*pi);

		// TODO: if a par file is specified, base it on the number of par file rotations
		size_t numPoints = vm["numpoints"].as<size_t>();

		string sOutputPrefix = vm["output"].as<string>();
		path pOutputPrefix(sOutputPrefix);
		path pOutRuns(pOutputPrefix);
		string sOutputTable;
		if (createRun)
		{
			if (!exists(pOutRuns)) boost::filesystem::create_directory(pOutRuns);
			else throw debug::xBadInput("Output directory exists");
			path pSummary = pOutRuns / path("summary");
			if (!exists(pSummary)) boost::filesystem::create_directory(pSummary);
			sOutputTable = path(path(pSummary) / "rotationtable.tsv").string();

			// Also create a script that will execute the runs sequentially and then collect the outputs
			auto createRunScript = [&](const boost::filesystem::path &base, size_t n)
			{
				using namespace boost::filesystem;
				path pScript = base / "doruns.csh";
				ofstream os(pScript.string().c_str());
				os << "#!/bin/tcsh\n"
					"# Run script for random orientations\n"
					"module purge\n";
				if (vm.count("modules"))
					os << "module load " << vm["modules"].as<string>() << "\n";
				os << "\n"
					"\n"
					"set n=0\n"
					"while ($n < " << n << ")\n"
					"\tcd $n\n"
					"\tset OMP_NUM_THREADS=4\n"
					"\tddscat >& ddscat.log\n"
					"\tcp w000r000k000.sca ../summary/r${n}.sca\n"
					"\tcp w000r000k000.fml ../summary/r${n}.fml\n"
					"\tcp ddscat.log ../summary/r${n}.log\n"
					"\tcd ..\n"
					"\t@ n += 1\n"
					"end\n"
					"\n";
			};
			createRunScript(pOutRuns, numPoints);
			for (const path &p : vSkels)
				{
					path pa(p);
					if (p.empty()) continue;
					//if (p.filename() == "ddscat.par") continue;
					if (boost::filesystem::is_directory(p)) continue;
					path pLink = path(pSummary) / p.filename();
					try {
						boost::filesystem::create_hard_link(pa, pLink);
					} catch (std::exception &) {
						try {
							boost::filesystem::create_symlink(pa, pLink);
						} catch (std::exception &e) {
							throw e;
						}
					}
				}
		} else {
			sOutputTable = sOutputPrefix;
			sOutputTable.append(".tsv");
		}
		ofstream out(sOutputTable.c_str());

		out << "n\tx\ty\tz\tr\tBeta_D\tTheta_D\tPhi_D\tBeta_R\tTheta_R\tPhi_R\n";
		for (size_t i=0; i<numPoints; ++i)
		{
			vector<float> crdsCartesian = random_on_sphere();
			// Convert from cartesian coordinates to rotation angles (in degrees)
			// The norm of the input vector is unity.
			auto radToDeg = [&pi](float rad) -> float
			{
				float res = rad * 180.f / pi;
				return res;
			};

			float beta_R = distUni(rand_gen);
			float &r = beta_R;
			
			float theta_R = acos(crdsCartesian[2]);
			float phi_R = atan2f(crdsCartesian[1], crdsCartesian[0]) + pi;

			float beta_D = radToDeg(beta_R);
			float theta_D = radToDeg(theta_R);
			float phi_D = radToDeg(phi_R);

			crdsCartesian[0] *= r;
			crdsCartesian[1] *= r;
			crdsCartesian[2] *= r;

			out << i << "\t"
				<< crdsCartesian[0] << "\t" << crdsCartesian[1] << "\t"
				<< crdsCartesian[2] << "\t"
				<< r << "\t"
				<< beta_D << "\t" << theta_D << "\t" << phi_D << "\t"
				<< beta_R << "\t" << theta_R << "\t" << phi_R << "\n"
				;

			if (createRun)
			{
				path pRun(pOutRuns);
				pRun = pRun / boost::lexical_cast<string>(i);
				boost::filesystem::create_directory(pRun);
				for (const path &p : vSkels)
				{
					if (p.empty()) continue;
					if (p.filename() == "ddscat.par") continue;
					if (boost::filesystem::is_directory(p)) continue;
					path pLink = pRun / p.filename();
					try {
						boost::filesystem::create_hard_link(p, pLink);
					} catch (std::exception &) {
						try {
							boost::filesystem::create_symlink(p, pLink);
						} catch (std::exception &e) {
							throw e;
						}
					}
				}
				rtmath::ddscat::rotations rots(beta_D, beta_D, 1, 
					theta_D, theta_D, 1, 
					phi_D, phi_D, 1);
				parfile->setRots(rots);
				path pOutPar = pRun / "ddscat.par";
				parfile->writeFile(pOutPar.string());
			}
		}
		

	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

