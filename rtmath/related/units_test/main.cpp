#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/splitSet.h>

#include <udunits2.h>

int main(int argc, char** argv) {
	try {
		using namespace std;
		using boost::shared_ptr;
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		Ryan_Debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<float>()->default_value(1.f), "input value")
			("units,u", po::value<string>(), "input units")
			("output,o", po::value<string>(), "output units")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		Ryan_Debug::process_static_options(vm);

		auto doHelp = [&](const std::string& s)
		{
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() < 2) doHelp("");
		float input = vm["input"].as<float>();
		if (!vm.count("units")) doHelp("Input units required");
		string inunits = vm["units"].as<string>();
		if (!vm.count("output")) doHelp("Output units required");
		string output = vm["output"].as<string>();

		shared_ptr<ut_system> utsys(ut_read_xml(nullptr), ut_free_system);
		if (!utsys) { throw; }

		shared_ptr<ut_unit> inunit(
			ut_parse(utsys.get(),inunits.c_str(),UT_ASCII), ut_free);
		if (!inunit) { cerr << "Cannot parse input units" << endl; throw; }
		shared_ptr<ut_unit> outunit(
			ut_parse(utsys.get(),output.c_str(),UT_ASCII), ut_free);
		if (!outunit) { cerr << "Cannot parse output" << endl; throw; }
		int can_convert = ut_are_convertible(inunit.get(), outunit.get());
		if (!can_convert) { cerr << "Cannot convert units" << endl; throw; }
		shared_ptr<cv_converter> converter(ut_get_converter(inunit.get(), outunit.get()), cv_free);
		if (!converter) { cerr << "Cannot convert create converter" << endl; throw; }
		float out = cv_convert_float(converter.get(), input);

		cout << out << " " << output << endl;
	} catch (std::exception &e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}

