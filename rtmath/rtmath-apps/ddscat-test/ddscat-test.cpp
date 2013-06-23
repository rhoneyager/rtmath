/* This program is designed to compare two ddscat output files (typically .avg) and report 
 * changes that exceed a given threshold.
 */
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <Ryan-Debug/debug.h>
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-ddscat-test\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("base,b", po::value<string>(),"specify base file (the reference values)")
			("input,i", po::value<string>(), "specify input file (checking against these values)")
			("tolerance,t", po::value<double>()->default_value(5), "Percent tolerance for value checks")
			("tolerance-abs,a", po::value<double>()->default_value(1.e-6), "Tolerance for near-zero quantities")
			;

		po::positional_options_description p;
		p.add("base",1);
		p.add("input",2);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << message << endl;
			cerr << desc << endl;
			exit(1);
		};
		if (vm.count("help") || argc == 1) doHelp("");
		double tolerance, toleranceAbs;
		string sInput, sBase;
		if (vm.count("input")) sInput = vm["input"].as<string>();
		else doHelp("Need to specify input file");
		if (vm.count("base")) sBase = vm["base"].as<string>();
		else doHelp("Need to specify base file");

		tolerance = vm["tolerance"].as<double>();
		toleranceAbs = vm["tolerance-abs"].as<double>();

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

		// If mismatched file type, report failure.
		boost::filesystem::path pInput(sInput), pBase(sBase);
		if (pInput.extension() != pBase.extension())
		{
			std::cerr << "File types do not match!" << endl;
			exit(2);
		}

		using namespace rtmath::ddscat;
		ddOutputSingle ddInput(sInput), ddBase(sBase);

		// Check header strings
		ddOutputSingle::headerMap mapInput, mapBase;
		ddInput.getHeaderMaps(mapInput);
		ddBase.getHeaderMaps(mapBase);
		{
			auto it = mapInput.begin();
			auto ot = mapBase.begin();
			if (mapInput.size() != mapBase.size())
				throw rtmath::debug::xBadInput("Headers have different sizes!");
			while (it != mapInput.end() && ot != mapBase.end())
			{
				if (it->first != ot->first) 
				{
					// Unaligned header symbols
					throw rtmath::debug::xBadInput(
							"Files do not have the same header quantities");
				}
				if (it->second->operator!=(*(ot->second)))
				{
					// A mismatch has occurred
					numFailures++;
					std::cerr << "Header mismatch:\n\t";
					std::cerr << *(it->second) << "\t" << *(ot->second);
				}
				++it; ++ot;
			}
		}

		// Check the stat tables
		ddOutputSingle::statTableType statsInput, statsBase;
		ddInput.getStatTable(statsInput);
		ddBase.getStatTable(statsBase);
		for (size_t j=0; j<(size_t) rtmath::ddscat::NUM_STAT_ENTRIES; j++)
		{
			double b = statsInput[j];
			double a = statsBase[j];
			// Check for tolerance in percentage and in abs terms.
			// If both criteria fail, then report a failure.
			if (fTol(a,b)) continue;
			//if (abs( (a - b) / a)
			//		* 100 < tolerance) continue;
			//if (abs(a - b) < toleranceAbs) continue;
			// By here, a failure has occurred
			numFailures++;
			// Convert j to the appropriate stat table entry name and
			// report the failure.
			std::cerr << "Stat table mismatch in "
				<< getStatNameFromId( (rtmath::ddscat::stat_entries) j)
				<< " - " << a << " versus " << b << std::endl;
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
		{
			auto it = scattInput.begin();
			auto ot = scattBase.begin();
			while (it != scattInput.end() && ot != scattBase.end())
			{
				if ((*it)->id() != (*ot)->id())
					throw rtmath::debug::xBadInput("Scattering matrix type mismatch");
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
				if ((*it)->id() == rtmath::ddscat::P)
				{
					// Mueller matrix comparison
					auto mI = (*it)->mueller();
					auto mB = (*ot)->mueller();
					bool fail = false;
					// Matrices are 4x4 double
					for (size_t i=0; i<4 && !fail; i++)
					{
						for (size_t j=0; j<4 & !fail; j++)
						{
							if (!fTol(mI(i,j),mB(i,j))) fail = true;
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
				} else {
					// FML matrix comparison
					boost::shared_ptr<const ddScattMatrixF> sfi(
						boost::dynamic_pointer_cast<const ddScattMatrixF>(*it));
					boost::shared_ptr<const ddScattMatrixF> sfb(
						boost::dynamic_pointer_cast<const ddScattMatrixF>(*it));
					ddScattMatrix::FType fI = sfi->getF();
					ddScattMatrix::FType fB = sfb->getF();
					// Matrices are 2x2 complex double
					bool fail = false;
					for (size_t i=0; i<2 && !fail; i++)
					{
						for (size_t j=0; j<2 && !fail; j++)
						{
							if (!fTol(fI(i,j).real(),fB(i,j).real())) fail = true;
							if (!fTol(fI(i,j).imag(),fB(i,j).imag())) fail = true;
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
		} else {
			cout << "All tests passed!" << endl;
		}
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


