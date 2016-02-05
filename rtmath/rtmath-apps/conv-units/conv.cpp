/* conv-units is a program that tests and implements unit conversions with the rtmath library.
   This is more powerful than just standard unit conversions, as conversions relying on
   equations are also supported (for example, interconversions between frequency and wavelength)
   */

#include <iostream>
#include <set>
#include <string>
#include <map>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <Ryan_Debug/debug.h>
//#include <Ryan_Debug/splitSet.h>
#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/conversions/convertLength.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::units::keywords;
	try {
		// Process some of the flags

		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
		config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< double >(), "Input quantity")
			("input-units,u", po::value< string >(), "Input units")
			("output-units,o", po::value< string >(), "Output units")

			("input-length-type", po::value<string>(), "Input length type (Max_Dimension, "
			 "Effective_Radius, ...")
			("output-length-type", po::value<string>(), "Output length type")
			("aspect-ratio", po::value<double>()->default_value(1.0), "Aspect "
			 "ratio for length conversions")
			("input-vf", po::value<double>()->default_value(1.0),
			 "Input volume fraction")
			("output-vf", po::value<double>()->default_value(1.0),
			 "Output volume fraction")

			("spec", "Interconvert spectral units (frequency, wavelength, wavenumber)")
			;

		po::positional_options_description p;
		p.add("input",1);
		p.add("input-units",2);
		p.add("output-units",3);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		if (vm.count("help") || !vm.count("input") ||
			!vm.count("output-units") || !vm.count ("input-units") ) {
			cerr << desc << "\n";
			return 1;
		}

		double inVal, outVal;
		string inUnits, outUnits;

		inVal = vm["input"].as<double>();
		inUnits = vm["input-units"].as<string>();
		outUnits = vm["output-units"].as<string>();

		boost::shared_ptr<rtmath::units::converter> cnv;

		if (vm.count("spec")) {
			cnv = boost::shared_ptr<rtmath::units::converter>(new rtmath::units::conv_spec(inUnits,outUnits));
		} else {
			cnv = boost::shared_ptr<rtmath::units::converter>(new rtmath::units::converter(inUnits,outUnits));
		}
		outVal = cnv->convert(inVal);

		if (!vm.count("spec") &&
				vm.count("input-length-type") &&
				vm.count("output-length-type")) {
			string ilt = vm["input-length-type"].as<string>();
			string olt = vm["output-length-type"].as<string>();
			double ivf = vm["input-vf"].as<double>();
			double ovf = vm["output-vf"].as<double>();
			double ar = vm["aspect-ratio"].as<double>();
			using namespace rtmath::units;
			double cnvVal = 0;
			cnvVal = convertLength(
				_in_length_value = outVal,
				_in_length_type = ilt,
				_out_length_type = olt,
				_ar = ar,
				_in_volume_fraction = ivf,
				_out_volume_fraction = ovf);
			cout << cnvVal << endl;
		} else {
			cout << outVal << endl;
		}
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}
