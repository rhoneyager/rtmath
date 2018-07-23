#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <complex>
#include <vector>
#include "../headers/tmatrix.h"
#include <Ryan_Debug/debug.h>

int main(int argc , char** argv)
{
	try {
		using namespace std;
		using namespace tmatrix_random;
		namespace po = boost::program_options;

		po::options_description cmdline("Command-line options"), config("Config options"),
			hidden("Hidden options"), desc("Allowed options"), oall("all options");
		Ryan_Debug::add_options(cmdline, config, hidden);
		cmdline.add_options()
			("axi,a", po::value<double>()->default_value(100), "Radius of equivalent sphere")
			("rat,r", po::value<double>()->default_value(1), "1=v_sphere, !1=sa")
			("lam,l", po::value<double>()->default_value(1635.44), "Wavelength (um)")
			("mrr", po::value<double>()->default_value(1.783), "Real refractive index")
			("mri", po::value<double>()->default_value(0.003862), "Imag >0 refr index")
			("eps", po::value<double>()->default_value(1.00001), "AR (>1 oblate <1 prolate)")
			("np", po::value<int>()->default_value(-1), "Number of quadrature points?")
			("ddelt", po::value<double>()->default_value(0.001), "Accuracy step")
			("ndgs", po::value<int>()->default_value(2), "?")
			("help,h", "Display help")
			;
		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);
		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(oall).run(), vm);
		po::notify(vm);
		Ryan_Debug::process_static_options(vm);

		//Ryan_Debug::printDebugInfo();
		//Ryan_Serialization::printDebugInfo();
		//tmatrix_random::printDebugInfo();
		if (vm.count("help")) {
			std::cout << desc << std::endl;
			exit(3);
		}
		cout << "Constructing and invoking\n";
		const double pi = boost::math::constants::pi<double>();

		boost::shared_ptr<const tmatrix_random::tmatrixParams> params =
			::tmatrix_random::tmatrixParams::create(
			vm["axi"].as<double>(), //100, // AXI
			vm["rat"].as<double>(), //1, // RAT
			vm["lam"].as<double>(), //1635.44, // LAM
			vm["mrr"].as<double>(), //1.783, // MRR
			vm["mri"].as<double>(), //0.003862, // MRI
			vm["eps"].as<double>(), //1.00001, // EPS
			vm["np"].as<int>(), //-1, // NP
			vm["ddelt"].as<double>(), //0.001, // DDELT
			vm["ndgs"].as<int>() //2 // NDGS
			);
		//params->npna = 3;

		auto ot = ::tmatrix_random::OriTmatrix::calc(params);

		double aeffRat = 1; //scaledAeff / i.aeff;
		double aeffRatSq = aeffRat * aeffRat;

		cout << "Qsca " << ot->qsca <<
			"\nQext " << ot->qext <<
			"\nwalb " << ot->walb <<
			"\ng " << ot->g <<
			"\nbk " << ot->qbk * aeffRatSq << endl;
		cout << endl << endl;

		//auto ic = ::tmatrix_random::IsoAngleRes::calc(ot);
		//cout << "\tAngle\tP11" << endl;
		//for (const auto &i : ic->P) {
		//	cout << i.first; //<< "\t" << i.second.at(0) << "\n";
		//	for (size_t j=0; j<16; ++j) {
		//		if (j%4 == 0) cout << "\n\t";
		//		else cout << "\t";
		//		cout << i.second.at(j);
		//	}
		//	cout << endl;
		//}

		//cout << "Qbksc " << tmatrix_random::
		//	getDifferentialBackscatterCrossSectionUnpol
		//	(ot->iso_angle_res) << endl;
	} catch (std::exception &e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
