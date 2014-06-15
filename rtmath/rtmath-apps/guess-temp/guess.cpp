// Guesses temperature from refractive index for a given frequency and substance
#include <iostream>
#include <complex>
#include <string>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		// Process some of the flags

		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
		config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("re,r", po::value<double>(), "Real refractive index")
			("im,i", po::value<double>(), "Imaginary refractive index (unused)")
			("frequency,f", po::value<double>(), "Frequency (GHz)")
			;
		po::positional_options_description p;
		p.add("frequency",1);
		p.add("re",2);
		p.add("im",3);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		auto doHelp = [&](const std::string &m)
		{
			using namespace std;
			cerr << desc << endl;
			cerr << m << endl;
			exit(1);
		};
		if (vm.count("help")) doHelp("");
		if (!vm.count("frequency")) doHelp("Must specify frequency");
		if (!vm.count("re")) doHelp("Must specify real refractive index");
		if (!vm.count("im")) doHelp("Must specify imaginary refractive index");


		double mre = vm["re"].as<double>();
		double mim = vm["im"].as<double>();
		double f = vm["frequency"].as<double>();

		std::complex<double> m(mre,mim);
		double T = rtmath::refract::guessTemp(f, m);
		cout << T << " K" << endl;
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}
