#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <complex>
#include <vector>
#include "../headers/tmatrix.h"
//#include "../headers/tmatrix-serialization.h"
//#include "../headers/serialization.h"
#include <Ryan_Debug/debug.h>
//#include <Ryan_Serialization/serialization.h>

//#pragma comment(lib,"tmatrix-cpp")

int main(int argc , char** argv)
{
	try {
		using namespace std;
		using namespace tmatrix;
		namespace po = boost::program_options;

		po::options_description cmdline("Command-line options"), config("Config options"),
			hidden("Hidden options"), desc("Allowed options"), oall("all options");
		Ryan_Debug::add_options(cmdline, config, hidden);
		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);
		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(oall).run(), vm);
		po::notify(vm);
		Ryan_Debug::process_static_options(vm);

		Ryan_Debug::printDebugInfo();
		//Ryan_Serialization::printDebugInfo();
		tmatrix::printDebugInfo();
		

		cout << "Constructing and invoking\n";
		const double pi = boost::math::constants::pi<double>();

		boost::shared_ptr<const tmatrix::tmatrixParams> params = 
			::tmatrix::tmatrixParams::create(
			10, // AXI
			0.1, // RAT
			2.*pi, // LAM
			1.5, // MRR
			0.02, // MRI
			0.5, // EPS
			-1, // NP
			0.001, // DDELT
			2 // NDGS
			);
		boost::shared_ptr<const tmatrix::OriTmatrix> ot = 
			::tmatrix::OriTmatrix::calc(params,145,52);

		boost::shared_ptr<const tmatrix::OriAngleRes> ang = 
			::tmatrix::OriAngleRes::calc(ot,65,56,128,114);
		cout << "S" << endl;
		cout << ang->getS(0,0) << endl;
		cout << ang->getS(0,1) << endl;
		cout << ang->getS(1,0) << endl;
		cout << ang->getS(1,1) << endl;

		cout << endl << endl;
		//Ryan_Serialization::write(ang.get(),"test.xml","::tmatrix::OriAngleRes");


		ang = ::tmatrix::OriAngleRes::calc(ot,0,0,0,0);
		cout << "S" << endl;
		cout << ang->getS(0,0) << endl;
		cout << ang->getS(0,1) << endl;
		cout << ang->getS(1,0) << endl;
		cout << ang->getS(1,1) << endl;

		cout << endl << endl;


		cout << "Qbksc " << tmatrix::getDifferentialBackscatterCrossSectionUnpol(ot) << endl;

		
		
		//Ryan_Serialization::write(ang.get(),"test2.xml","::tmatrix::OriAngleRes");
	} catch (std::exception &e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
