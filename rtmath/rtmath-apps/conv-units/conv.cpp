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
#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/units.h"
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
			("input,i", po::value< double >(), "Input quantity")
			("input-units,u", po::value< string >(), "Input units")
			("output-units,o", po::value< string >(), "Output units")

			("density", "Convert density units")
			("mass", "Convert mass  units (kg, g, ...)")
			("length", "Convert length units (m, km, ft, ...)")
			("spec", "Interconvert spectral units (frequency, wavelength, wavenumber) (DEFAULT)")
			("volume","Convert units of volume (m^3, ...)")
			("pressure", "Convert units of pressure (Pa, hPa, atm)")

			("temperature", "Perform temperature conversion (K, C, F, R)");

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

		if (vm.count("density")) {
			cnv = boost::shared_ptr<rtmath::units::conv_dens>(new rtmath::units::conv_dens(inUnits,outUnits));
		} else if (vm.count("mass")) {
			cnv = boost::shared_ptr<rtmath::units::conv_mass>(new rtmath::units::conv_mass(inUnits,outUnits));
		} else if (vm.count("length")) {
			cnv = boost::shared_ptr<rtmath::units::conv_alt>(new rtmath::units::conv_alt(inUnits,outUnits));
		} else if (vm.count("spec")) {
			cnv = boost::shared_ptr<rtmath::units::conv_spec>(new rtmath::units::conv_spec(inUnits,outUnits));
		} else if (vm.count("volume")) {
			cnv = boost::shared_ptr<rtmath::units::conv_vol>(new rtmath::units::conv_vol(inUnits,outUnits));
		} else if (vm.count("pressure")) {
			cnv = boost::shared_ptr<rtmath::units::conv_pres>(new rtmath::units::conv_pres(inUnits,outUnits));
		} else if (vm.count("temperature")) {
			cnv = boost::shared_ptr<rtmath::units::conv_temp>(new rtmath::units::conv_temp(inUnits,outUnits));
		} else {
			cnv = boost::shared_ptr<rtmath::units::conv_spec>(new rtmath::units::conv_spec(inUnits,outUnits));
			//cerr << "Must specify a valid type of unit to convert (temperature, linear distance, ...)\n";
			//cerr << desc << endl;
			//return 1;
		}

		
		// Do only spectral conversions for now
		outVal = cnv->convert(inVal);
		cout << outVal << endl;
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}
