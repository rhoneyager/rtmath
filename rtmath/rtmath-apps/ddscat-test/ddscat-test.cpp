/* This program is designed to compare two ddscat output files (typically .avg) and report 
 * changes that exceed a given threshold.
 */
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

enum matchCriteria
{
	MATCH_STANDARD,
	MATCH_FOLDERS,
	MATCH_DIPOLES
};

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-ddscat-test\n\n";
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("base,b", po::value<vector<string> >(),"specify base files (the reference values)")
			("input,i", po::value<vector<string> >(), "specify input files (checking against these values)")
			("tolerance,t", po::value<double>()->default_value(5), "Percent tolerance for value checks")
			("tolerance-abs,a", po::value<double>()->default_value(1.e-6), 
			"Tolerance for near-zero quantities")
			("check-header", po::value<bool>()->default_value(false),
			"Fail on header (polarization, solution method, ...) differences")
			("check-basic", po::value<bool>()->default_value(true),
			"Enforce a check on the number of dipoles and the effective radius")
			("check-results", po::value<bool>()->default_value(true),
			"Match the Mueller matrices and stats table. Turn this off when just "
			"verifying correct dipole number and effective radius. Used when there "
			"is no base result at the given frequency")
			("match-by-folder", "Instead of matching by standard prefix, match by containing folder")
			("match-by-dipoles", "Instead of matching by a standard prefix, match avg files and shape "
			"files by the number of dipoles. Necessary for Liu raw pristine flake extraction. "
			"Will use a default par file this way.")
			;

		po::positional_options_description p;
		p.add("base",1);
		p.add("input",2);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << message << endl;
			cerr << desc << endl;
			exit(1);
		};
		if (vm.count("help") || argc == 1) doHelp("");
		double tolerance, toleranceAbs;
		vector<string> sInput, sBase;
		if (vm.count("input")) sInput = vm["input"].as<vector<string> >();
		else doHelp("Need to specify input files");
		if (vm.count("base")) sBase = vm["base"].as<vector<string> >();
		else doHelp("Need to specify base files");

		tolerance = vm["tolerance"].as<double>();
		toleranceAbs = vm["tolerance-abs"].as<double>();

		matchCriteria mc = MATCH_STANDARD;
		if (vm.count("match-by-folder")) mc = MATCH_FOLDERS;
		if (vm.count("match-by-dipoles")) mc = MATCH_DIPOLES;

		bool checkBasic = vm["check-basic"].as<bool>();
		bool checkHeader = vm["check-header"].as<bool>();
		bool checkResults = vm["check-results"].as<bool>();

		auto makePathAbsolute = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (p.is_absolute()) return p;
			path pp = boost::filesystem::absolute(p);
			return pp;
		};

		using boost::filesystem::path;

		std::map<std::string, rtmath::ddscat::dataset> maps;

		using rtmath::ddscat::dataset;
		auto getPrefix = [&](const path &p, std::string &prefix) -> bool
		{
			if (mc == MATCH_STANDARD)
				prefix = dataset::getPrefix(p.string());
			else if (mc == MATCH_FOLDERS) {
				prefix = makePathAbsolute(p).string();
				prefix = path(prefix).remove_filename().string();
			}
			else if (mc == MATCH_DIPOLES) {
				// If a shape file, read shape and extract dipole number
				size_t nDipoles = 0;
				double freq = 0;
				std::complex<double> m;
				if (dataset::isShape(p)) {
					rtmath::ddscat::shapefile::shapefile s;
					s.readHeaderOnly(p.string());
					nDipoles = s.numPoints;
				}
				// If an avg file, read and extract dipole number.
				else if (dataset::isAvg(p)) {
					rtmath::ddscat::ddOutputSingle a;
					a.readFile(p.string(), ".avg");
					nDipoles = a.numDipoles();
					freq = a.freq();
					m = a.getM();
				}
				if (!nDipoles) return false; // Prefix 0 is not valid. Wrong file type.
				std::ostringstream ntos;
				ntos << nDipoles; // dipole number matching
				ntos << "-" << std::setprecision(4) << m.real() // freq + temperature matching
					<< "-" << std::setprecision(4) << m.imag()
					<< "-" << std::setprecision(2) << freq; // frequency matching
				prefix = ntos.str();
				//prefix = makePathAbsolute(path(it)).string();
				//prefix = path(prefix).remove_filename().string();
			}
			if (prefix.size() == 0) return false;
			return true;
		};
		auto matchIDS = [&](const vector<path> &v)
		{
			for (auto &it : v)
			{
				string prefix;
				path pfile(it);
				
				if (!getPrefix(pfile, prefix)) continue;

				cerr << "prefix: " << prefix << " for file " << it << endl;
				if (!maps.count(prefix))
				{
					maps[prefix] = dataset(prefix);
					cerr << "Adding prefix " << prefix << " from file " << it << endl;
				}
				if (!exists(pfile)) continue;
				//if (!dataset::isValid(pfile)) continue;
				if (dataset::isAvg(pfile)) {
					maps[prefix].ddres.push_back(pfile);
					cerr << "\tMatched as avg file\n";
				}
				else if (dataset::isPar(pfile) ||
					pfile.string().find("par") != std::string::npos) {
					maps[prefix].parfile = pfile;
					cerr << "\tMatched as par file\n";
				}
				else if (!dataset::isValid(pfile)) continue;
				else if (dataset::isShape(pfile)) {
					// TODO: shape file match can catch par files. It is 
					// too loose.
					maps[prefix].shapefile = pfile;
					cerr << "\tMatched as shape file\n";
				}
			}
		};

		auto expandFolders = [&](const vector<string> &src, vector<path> &dest)
		{
			dest.clear();
			for (auto s : src)
			{
				using namespace boost::filesystem;
				path p(s);
				if (is_directory(p))
					copy(directory_iterator(p),
					directory_iterator(), back_inserter(dest));
				else dest.push_back(p);
			}
		};

		vector<path> inputs, refs;
		expandFolders(sInput, inputs);
		expandFolders(sBase, refs);

		matchIDS(refs);

		// Now, iterate through all input files to be compared against, 
		// extract the prefixes, and select the reference file for comparison.
		// 

		// Tolerance evaluator
		auto fTol = [tolerance,toleranceAbs](double a, double b)->bool
		{
			if (abs(a) < toleranceAbs && abs(b) < toleranceAbs)
			{
				if (abs(a-b) > toleranceAbs) return false;
				return true;
			} else {
				if ( abs((a-b)/b)*100. > tolerance) return false;
				return true;
			}
		};

		int numFailures = 0;

		for (const auto &pInput : inputs)
		{
			cout << "Processing input file: " << pInput << endl;
			// Get input file prefix
			string prefix;
			if (!getPrefix(pInput, prefix))
			{
				cout << "\tCould not determine input file prefix.\n";
				continue;
			}
			cout << "\tHas prefix " << prefix << endl;
			// Find a matching base
			auto iBase = maps.cbegin();
			iBase = std::find_if(maps.cbegin(), maps.cend(),
				[&](const std::pair<std::string, dataset> &pr) -> bool
			{
				if (pr.first == prefix) return true;
				return false;
			});
			if (iBase == maps.cend())
			{
				cout << "\tCould not match prefix\n";
				continue;
			}
			if (iBase->second.ddres.size() == 0)
			{
				cout << "\tMatched prefix does not have and .avg,"
					" .fml or .sca files. Should never happen.\n";
				continue;
			}
			// If mismatched file type, report failure.
			boost::filesystem::path pBase(*(iBase->second.ddres.begin()));
			/* // Have to match liu avg_ files
			if (pInput.extension() != pBase.extension())
			{
				std::cout << "\tFile types do not match!" << endl;
				cout << "\tMatching file: " << pInput << endl
					<< "\tBase file: " << pBase << endl;
				continue;
			}
			*/

			using namespace rtmath::ddscat;
			ddOutputSingle ddInput(pInput.string()), ddBase(pBase.string());

			// Check header strings
			ddOutputSingle::headerMap mapInput, mapBase;
			ddInput.getHeaderMaps(mapInput);
			ddBase.getHeaderMaps(mapBase);
			if (checkHeader)
			{
				auto it = mapInput.begin();
				auto ot = mapBase.begin();
				if (mapInput.size() != mapBase.size())
				{
					cerr << "\tHeaders have different sizes!\n";
					continue;
				}
				while (it != mapInput.end() && ot != mapBase.end())
				{
					if (it->first != ot->first)
					{
						// Unaligned header symbols
						cerr << "\tUnaligned header symbols detected. Files do not have the same "
							"header quantities.\n";
						continue;
					}
					if (it->second->operator!=(*(ot->second)))
					{
						// A mismatch has occurred
						numFailures++;
						std::cerr << "Header mismatch:\n\t";
						//std::cerr << *(it->second) << "\t" << *(ot->second);
						/// \todo Fix the ddOutputSingleObj operator<< definition.
						it->second->write(std::cerr);
						ot->second->write(std::cerr);
					}
					++it; ++ot;
				}
			}

			if (checkBasic)
			{
				// Do an additional check against the effective radius and number of dipoles
				if (!fTol(ddInput.aeff(), ddBase.aeff()))
				{
					cerr << "Effective radius mismatch - base: " << ddBase.aeff() << ", input: " << ddInput.aeff() << endl;
					continue;
				}
				if (ddInput.numDipoles() != ddBase.numDipoles())
				{
					cerr << "Dipole count mismatch - base: " << ddBase.numDipoles() << ", input: " << ddInput.numDipoles() << endl;
					continue;
				}
			}

			// Check the stat tables
			ddOutputSingle::statTableType statsInput, statsBase;
			ddInput.getStatTable(statsInput);
			ddBase.getStatTable(statsBase);
			if (checkResults)
			{
				for (size_t j = 0; j < (size_t)rtmath::ddscat::NUM_STAT_ENTRIES; j++)
				{
					double b = statsInput[j];
					double a = statsBase[j];
					// Check for tolerance in percentage and in abs terms.
					// If both criteria fail, then report a failure.
					if (fTol(a, b)) continue;
					//if (abs( (a - b) / a)
					//		* 100 < tolerance) continue;
					//if (abs(a - b) < toleranceAbs) continue;
					// By here, a failure has occurred
					numFailures++;
					// Convert j to the appropriate stat table entry name and
					// report the failure.
					std::cerr << "Stat table mismatch in "
						<< getStatNameFromId((rtmath::ddscat::stat_entries) j)
						<< " - " << a << " versus " << b << std::endl;
				}
			}

			// Check FML / Mueller matrices
			// Get collections of scattering matrices
			// Iterate through each collection. Check that type and values are the same.
			// Ordering should have been preserved.
			ddOutputSingle::scattMatricesContainer scattInput, scattBase;
			ddInput.getScattMatrices(scattInput);
			ddBase.getScattMatrices(scattBase);
			/*
			for (auto &jt : scattInput)
			{
			std::cerr << jt->theta() << "\t" << jt->phi() << "\t" << jt->pol() << endl;
			}
			return 0;
			*/
			if (checkResults)
			{
				auto it = scattInput.begin();
				auto ot = scattBase.begin();
				while (it != scattInput.end() && ot != scattBase.end())
				{
					if ((*it)->id() != (*ot)->id())
					{
						cerr << "Scattering matrix type mismatch (FML vs SCA)" << endl;
						continue;
					}
					// Check freq, theta, phi, pol
					//std::cerr << "\tpol: " << (*it)->pol() << "\t" << (*ot)->pol() << "\t" 
					//	<< abs( ((*it)->pol() - (*ot)->pol()) / (*ot)->pol()) * 100. << std::endl;
					if (!(*it)->compareTolHeader((**ot), tolerance))
					{
						std::cerr << "First part of matrix entry exceeds tolerances.\n"
							<< "\ttheta: " << (*ot)->theta() << "\t" << (*it)->theta() << "\n"
							<< "\tphi: " << (*ot)->phi() << "\t\t" << (*it)->phi() << "\n"
							<< "\tpol: " << (*ot)->pol() << "\t" << (*it)->pol() << "\n";
						numFailures++;
						++it; ++ot;
						continue;
					}

					// Cast to correct subtype and compare entries
					if ((*it)->id() == rtmath::ddscat::scattMatrixType::P)
					{
						// Mueller matrix comparison
						auto mI = (*it)->mueller();
						auto mB = (*ot)->mueller();
						bool fail = false;
						// Matrices are 4x4 double
						for (size_t i = 0; i < 4 && !fail; i++)
						{
							for (size_t j = 0; j < 4 && !fail; j++)
							{
								if (!fTol(mI(i, j), mB(i, j))) fail = true;
							}
						}
						if (fail)
						{
							numFailures++;
							std::cerr << "Matrix entry exceeds tolerance: "
								<< "theta: " << (*ot)->theta()
								<< " phi: " << (*ot)->phi()
								<< " pol: " << (*ot)->pol() << "\n"
								<< mI << "\n\n" << mB << endl;
						}
					}
					else {
						// FML matrix comparison
						boost::shared_ptr<const ddScattMatrixF> sfi(
							boost::dynamic_pointer_cast<const ddScattMatrixF>(*it));
						boost::shared_ptr<const ddScattMatrixF> sfb(
							boost::dynamic_pointer_cast<const ddScattMatrixF>(*it));
						ddScattMatrix::FType fI = sfi->getF();
						ddScattMatrix::FType fB = sfb->getF();
						// Matrices are 2x2 complex double
						bool fail = false;
						for (size_t i = 0; i < 2 && !fail; i++)
						{
							for (size_t j = 0; j < 2 && !fail; j++)
							{
								if (!fTol(fI(i, j).real(), fB(i, j).real())) fail = true;
								if (!fTol(fI(i, j).imag(), fB(i, j).imag())) fail = true;
							}
						}
						if (fail)
						{
							numFailures++;
							std::cerr << "Matrix entry exceeds tolerance: "
								<< "theta: " << (*ot)->theta()
								<< " phi: " << (*ot)->phi()
								<< " pol: " << (*ot)->pol() << "\n"
								<< fI << "\n\n" << fB << endl;
						}
					}
					++it; ++ot;
				}
				if (it != scattInput.end() || ot != scattBase.end())
				{
					std::cerr << "Matrix size mismatch. Input has " << scattInput.size()
						<< " entries, whereas base has " << scattBase.size() << " entries.\n";
					numFailures++;
				}
			}

			if (numFailures)
			{
				cout << "There were " << numFailures << " failures." << endl;
				return numFailures;
			}
			else {
				cout << "All tests passed!" << endl;
			}
		}
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


