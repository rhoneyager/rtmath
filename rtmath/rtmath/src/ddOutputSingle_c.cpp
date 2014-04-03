#include "Stdafx-ddscat.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/qi_omit.hpp>
#include <boost/spirit/include/qi_repeat.hpp>

#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/units.h"
#include "../rtmath/macros.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"



/// Internal namespace for the reader parsers
namespace {
	namespace qi = boost::spirit::qi;
	namespace ascii = boost::spirit::ascii;
	namespace phoenix = boost::phoenix;

	/** \brief Parses space-separated numbers.
	*
	* \see rtmath::ddscat::ddOutputSingle::readF
	* \see rtmath::ddscat::ddOutputSingle::readMueller
	**/
	template <typename Iterator>
	bool parse_numbers_space(Iterator first, Iterator last, std::vector<double>& v)
	{
		using qi::double_;
		using qi::phrase_parse;
		using qi::_1;
		using ascii::space;
		using phoenix::push_back;

		bool r = phrase_parse(first, last,

			//  Begin grammar
			(
			*double_
			)
			,
			//  End grammar

			space, v);

		if (first != last) // fail if we did not get a full match
			return false;
		return r;
	}
}

namespace rtmath {

	namespace ddscat {

		void ddOutputSingle::readMueller(std::istream &in)
		{
			using namespace std;
			// The frequency is needed when reading this matrix
			double freq = 0;
			if (wave())
				freq = units::conv_spec("um", "GHz").convert(wave());

			string lin;
			mMuellerIndices &mIndices = _muellerMap;
			mIndices.clear();
			vector<double> vals;
			vals.reserve(10);

			while (in.good())
			{
				std::getline(in, lin);
				// Parse the string to get rid of spaces. This is used to determine 
				// if we are still in the S matrix header or in the actual data
				boost::trim(lin);
				if (!lin.size()) continue;
				//std::cerr << lin << std::endl;
				// TODO: parse the header line to get the list of matrix entries known
				// TODO: use symmetry relationships in a depGraph to get the other 
				// mueller matrix entries.

				// Expecting the first line to begin with theta phi Pol. ...
				if (std::isalpha(lin.at(0)))
				{
					typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
					boost::char_separator<char> sep("\t ");
					tokenizer t(lin, sep);
					size_t i = 0; // Column number
					for (auto it = t.begin(); it != t.end(); ++it, ++i)
					{
						//std::cerr << "\t i: " << i << " it: " << *it << std::endl;
						// Mueller entry columns have a '_'
						size_t loc = it->find("_");
						if (loc == string::npos) continue;
						//std::cerr << it->substr(loc+1) << std::endl;
						size_t id = (size_t)macros::m_atoi(it->substr(loc + 1).c_str());
						size_t row = (id / 10) - 1; // Annoying start at 1...
						size_t col = (id % 10) - 1;
						//std::cerr << "mIndices loc: " << loc << " id: " << id << " i: " << i << " row: " << row << " col: " << col << std::endl;
						mIndices[i] = std::pair<size_t, size_t>(row, col);
					}

					// TODO: add function that generates the correct mueller relations from here
#pragma message("Warning: ddOutputSingle needs the Mueller matrix filling routine")
				}
				else {
					// Parse the Mueller entries
					//std::cerr << "Parsing " << lin << std::endl;
					// TODO: check this
					// The ordering is theta, phi, polarization, and then the 
					// relevant matrix entries
					// theta phi Pol. S_11 S_12 S_21 S_22 S_31 S_41
					vals.clear();
					if (!parse_numbers_space(lin.begin(), lin.end(), vals))
						throw debug::xBadInput("Cannot parse Mueller entry");

					//for (auto it = t.begin(); it != t.end(); ++it)
					//	vals.push_back(rtmath::macros::m_atof(it->data(), it->size())); // Speedup using my own atof
					//vals.push_back(boost::lexical_cast<double>(*it));
					// ddScattMatrixF constructor takes frequency (GHz) and phi
					boost::shared_ptr<ddScattMatrixP> mat(new ddScattMatrixP(freq, vals[0], vals[1]));
					ddScattMatrix::PnnType P;

					for (auto ot = mIndices.begin(); ot != mIndices.end(); ++ot)
					{
						P(ot->second.first, ot->second.second) = vals[ot->first]; // See Mueller header read
					}
#pragma message("Warning: ddOutputSingle needs the Mueller matrix filling routine (part b)")
					mat->setP(P);
					mat->polLin(vals[2]);

					boost::shared_ptr<const ddScattMatrix> matC =
						boost::dynamic_pointer_cast<const ddScattMatrix>(mat);

					_scattMatricesRaw.insert(matC);
					//std::cerr << _scattMatricesRaw.size() << " elements\n";
				}
			}

			_statTable_Size_ts.at(stat_entries_size_ts::NUMP) = _scattMatricesRaw.size();
		}



		void ddOutputSingle::readF(std::istream &in,
			boost::shared_ptr<const ddScattMatrixConnector> eProvider)
		{
			using namespace std;
			// The frequency is needed when reading this matrix
			double freq = 0;
			if (wave())
				freq = units::conv_spec("um", "GHz").convert(wave());

			string lin;

			std::vector<double> vals;
			vals.reserve(10);

			while (in.good())
			{
				std::getline(in, lin);
				if (lin == "") return;
				// Parse the string to get rid of spaces. This is used to determine
				// if we are still in the S matrix header or in the actual data
				boost::trim(lin);
				if (std::isalpha(lin.at(0))) continue;

				vals.clear();
				if (!parse_numbers_space(lin.begin(), lin.end(), vals))
					throw debug::xBadInput("Cannot parse F entry");

				// ddScattMatrixF constructor takes frequency (GHz) and phi
				boost::shared_ptr<ddScattMatrixF> mat(new ddScattMatrixF
					(freq, vals[0], vals[1], 0, 0, eProvider));
				ddScattMatrix::FType fs;
				fs(0, 0) = complex<double>(vals[2], vals[3]);
				fs(1, 0) = complex<double>(vals[4], vals[5]);
				fs(0, 1) = complex<double>(vals[6], vals[7]);
				fs(1, 1) = complex<double>(vals[8], vals[9]);
				mat->setF(fs);

				boost::shared_ptr<const ddScattMatrix> matC =
					boost::dynamic_pointer_cast<const ddScattMatrix>(mat);

				_scattMatricesRaw.insert(matC);
			}

			_statTable_Size_ts.at(stat_entries_size_ts::NUMF) = _scattMatricesRaw.size();
		}
	}
}
