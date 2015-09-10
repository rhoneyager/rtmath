// This program takes the csv version of the stata output and
// performs a psd integration of backscatter and scattering
// cross-sections. It is given the aeff_ice column and a range
// of columns to integrate over, and it performs the integration
// using the Sekhon and Srivastava [1970] size distribution.
#define BOOST_SPIRIT_USE_PHOENIX_V3

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
//#include <boost/spirit/include/karma.hpp>
//#include <boost/spirit/include/qi.hpp>
//#include <boost/spirit/include/qi_string.hpp>
//#include <boost/spirit/include/phoenix_core.hpp>
//#include <boost/spirit/include/phoenix_operator.hpp>
//#include <boost/spirit/include/phoenix_stl.hpp>
#include <fstream>
#include <complex>
#include <set>
#include <vector>
#include <sstream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging.h>
#include <Ryan_Debug/macros.h>
#include <boost/log/sources/global_logger_storage.hpp>
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/conversions/convertLength.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/config.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/error/debug.h"

BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
	m_app,
	boost::log::sources::severity_channel_logger_mt< >,
	(boost::log::keywords::severity = Ryan_Debug::log::error)
	(boost::log::keywords::channel = "app"));

#undef FL
#undef mylog
#define FL "main.cpp, line " << (int)__LINE__ << ": "
#define mylog(x) { std::ostringstream l; l << FL << x; BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_1) << l.str(); }

int main(int argc, char *argv[])
{
	using namespace std;
	using namespace rtmath::units::keywords;
	using rtmath::units::convertLength;
	using namespace rtmath::config;
	using namespace rtmath;
	try {
		cerr << "rtmath-integ-paper-1" << endl;
		const double pi = boost::math::constants::pi<double>();
		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("input,i", po::value<string>(), "The input file (csv format)")
			("range", po::value<string>(), "The set of the columns to be integrated over")
			("iceaeffum,x", po::value<size_t>(), "The x column (aeff_ice_um). Starts at 1.")
			//("keep,k", po::value<string>(), "Columns to keep")
			//("tag,t", po::value<string>(), "An identifying tag column (optional)")
			("output,o", po::value<string>(), "Output file")
			("rainrates,r", po::value<string>(), "The rainfall rates to consider (in mm/hr)")
			("help,h", "produce help message")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		auto& lg = m_app::get();
		// Begin checking parameters
		auto doHelp = [&](const std::string& s)
		{
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() == 0) doHelp("");

		vector<string> entries;
		entries.reserve(1100000 * 30);
		vector<string> header;
		size_t numLineCols = 0;
		// Read in everything as strings separated by commas.
		string lin;
		if (!vm.count("input")) doHelp("Input file needed");
		if (!vm.count("range")) doHelp("Data columns needed");
		if (!vm.count("rainrates")) doHelp("Need to specify rainfall rates");
		if (!vm.count("iceaeffum")) doHelp("Need column number for iceaeffum");
		if (!vm.count("output")) doHelp("Output file needed");
		string srange = vm["range"].as<string>();
		string srainrates = vm["rainrates"].as<string>();
		size_t cice = vm["iceaeffum"].as<size_t>();
		string output = vm["output"].as<string>();
		string input = vm["input"].as<string>();
		paramSet<size_t> crange(srange);
		paramSet<double> rainrates(srainrates);
		ifstream in(input.c_str());
		size_t line=0;
		while (in.good()) {
			std::getline(in,lin);
			if (!in.good()) break;
			if (!lin.size()) continue;
			if (line == 0) {
				//parse_strings_commas(lin.begin(), lin.end(), header);
				boost::split(header, lin, boost::is_any_of(",\t"));
				numLineCols = header.size();
			} else {
				//parse_strings_commas(lin.begin(), lin.end(), entries);
				vector<string> se;
				boost::split(se, lin, boost::is_any_of(",\t"));
				entries.insert(entries.end(), se.begin(), se.end());
			}
			line++;
		}

		size_t contentLines = line - 1;
		// Assume that the data is ordered by effective radius. Assume frequencies are
		// in separate files.
		vector<double> ranges( (line - 1) * crange.size());
		vector<double> PDFs( (line-1) * crange.size() * rainrates.size());
		vector<double> CDFs( (line-1) * crange.size() * rainrates.size());
		vector<double> diams( (line-1) ), iceaeffs(line-1);
		auto pIndex = [&crange, &rainrates, &contentLines](size_t i, size_t ln, size_t nrr) -> size_t {
			size_t res = (rainrates.size() - nrr - 1);
			res *= crange.size();
			res += (crange.size() - i - 1);
			res *= contentLines;
			res += ln;
			return res;
			//return i + (crange.size() * ln)
			//		+ (rainrates.size() * crange.size() * ln);
		};
		//vector<string> rangesNames(crange.size());
		// Convert source strings. Use my macros for speed.
		std::cerr << "Total lines: " << line << ", Content lines: " << contentLines << std::endl
			<< "numLineCols: " << numLineCols << ", crange.size(): " << crange.size() << std::endl
			<< "ranges.size(): " << ranges.size() << ", diams.size(): " << diams.size() << std::endl;
		std::cerr << "PDFs.size(): " << PDFs.size() << std::endl;
		for (size_t ln = 0; ln < contentLines; ln++) {
			size_t base = numLineCols * ln;
			//std::cout << "Line " << ln << ", base " << base << std::endl;
			// Aeff
			double aeff_ice = Ryan_Debug::macros::m_atof<double>(
				entries.at(base+cice-1).c_str());
			double mass = 0.9182 * pi * pow(aeff_ice,3.) * 4./3.;
			double vwater = mass / 0.9981;
			double aeff_water = pow(3.*vwater/(4.*pi),1./3.);
			double deff_water = 2. * aeff_water;
			diams.at(ln) = deff_water;
			iceaeffs.at(ln) = aeff_ice;
			// Sources
			size_t i=0;
			for (auto it = crange.begin(); it != crange.end(); ++it) {
				double src = Ryan_Debug::macros::m_atof<double>(entries.at(base-1+(*it)).c_str());
				size_t baseOut = crange.size() * ln;
				ranges.at(baseOut + i) = src;

				size_t nrr = 0;
				for (auto rr = rainrates.begin(); rr != rainrates.end(); ++rr) {
					double lambda = 22.9 * pow(*rr,-0.45);
					double N0 = 2500. * pow(*rr,-0.94);
					double Nd = N0 * exp(-1. * lambda * deff_water / 1.e4);
					double pdf = Nd  * src;
					if (pdf < 0) std::cerr << "pdf < 0! lam " << lambda << " n0 " << N0
						<< " nd " << Nd << " dwat " << deff_water << " src " << src << " pdf " << pdf << std::endl;
					//std::cout << "at line " << ln << " and i " << i
					//	<< " and nrr " << nrr << ", pIndex is "
					//	<< pIndex(i,ln,nrr) << " (diam " << diams.at(ln)
					//	<< " lam " << lambda
					//	<< " N0 " << N0 << " Nd " << Nd
					//	<< " src " << src << ")" << std::endl;
					//std::cout << "\tpdf is " << pdf;

					PDFs.at(pIndex(i,ln,nrr)) = pdf;
					if (ln > 0) {
						// Just use the trapeziod rule.
						double dw = deff_water - diams.at(ln-1);
						if (dw < 0) {
							std::cerr << "Flake results are not in order!!!!\n"
								<< "ln " << ln << " nrr " << nrr << " i " << i << std::endl;
						}
						double pdfprev = PDFs.at(pIndex(i,ln-1,nrr));
						if (pdfprev < 0) std::cerr << "pdfprev < 0. pdfprev is " << pdfprev << " at i " << i
							<< " ln " << ln - 1 << " nrr " << nrr << " for lambda " << lambda 
							<< " n0 " << N0 << " nd " << Nd << " pdf " << pdf << " dwat " << deff_water
							<< " src " << src << " rr " << *rr << " col " << *it << std::endl;
						double dP = ((pdf + pdfprev) * dw / 2.);
						// Safety check
						if (dP >= 0)
							CDFs.at(pIndex(i,ln,nrr)) = CDFs.at(pIndex(i,ln-1,nrr)) + dP;
						else {
							CDFs.at(pIndex(i,ln,nrr)) = CDFs.at(pIndex(i,ln-1,nrr));
							std::cerr << "dp < 0 at ln " << ln << " nrr " << nrr << " i " << i << std::endl;
							std::cerr << "dp " << dP << " dw " << dw << " pdf " << pdf << " pdfprev " << pdfprev
								<< " lam " << lambda << " rr " << *rr << " N0 " << N0 << " Nd " << Nd
								<< " dwat " << deff_water << " src " << src << " col " << *it << std::endl;
						}
						//std::cout << "\tdw: " << dw << " prevPDF: " << pdfprev
						//	<< " prevCDF " << CDFs.at(pIndex(i,ln-1,nrr))
						//	<< " PCDF is " << CDFs.at(pIndex(i,ln,nrr));
					}
					//std::cout << std::endl;
					nrr++;
				}
				++i;
			}

		}
		ofstream out(output.c_str());
		// Write the header
		out << "r";
		for (auto it = crange.begin(); it != crange.end(); ++it) {
			out << ",i" << header.at((*it)-1);
		}
		out << std::endl;
		// Write the values
		size_t nrr = 0;
		for (auto rr = rainrates.begin(); rr != rainrates.end(); ++rr) {
			out << *rr;
			for (size_t i=0; i<crange.size(); ++i) {
				out << "," << CDFs.at(pIndex(i, contentLines-1, nrr));
			}
			out << std::endl;
			nrr++;
		}
		/*
		out << "Aeff_ice,Deff_water";
		for (auto it = crange.begin(); it != crange.end(); ++it) {
			out << "," << header.at((*it)-1);
		}
		out << std::endl;
		// Write the values
		for (size_t i=0; i<diams.size(); ++i) {
			out << iceaeffs.at(i) << "," << diams.at(i);
			for (size_t j=0; j < crange.size(); ++j)
				out << "," << ranges.at(j+(i*crange.size()));
			out << std::endl;
		}
		*/
	}
	catch (std::exception &e) {
		cerr << "Exception: " << e.what() << endl;
		exit(2);
	}
	catch (...) {
		cerr << "Caught unidentified error... Terminating." << endl;
		exit(1);
	}

	return 0;
}



