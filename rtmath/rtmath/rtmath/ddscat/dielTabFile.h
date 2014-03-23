#pragma once
#include "../defs.h"
#include "../hash.h"
#include <complex>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

namespace rtmath
{
	namespace ddscat
	{
		/// Container for the enum class colMap
		namespace dielColumns
		{
			enum colMap
			{
				WAVELENGTH,
				M_RE,
				M_IM,
				E_RE,
				E_IM,
				NUMCOLS
			};
		}


		/** \brief Provides facilities for reading / writing 
		* diel.tab files, in both Liu and Draine forms. 
		*
		* The difference in the forms is that Liu's 
		* form just provides three points that give constant interpolation 
		* values for the dielectric constant of ice at constant 
		* temperature. The actual diel.tab behavior allows for a 
		* more-complete interpolation, which is what I will support.
		*
		* This class can read and write diel.tab files that match the 
		* code in ddscat. Each file has a header and can display frequency 
		* -dependent dielectrics using m or e. The reading and writing of 
		* these files is quite customizable, allowing for column reordering 
		* and suppression.
		**/
		class DLEXPORT_rtmath_ddscat_base dielTab
		{
		public:
			dielTab();
			dielTab(const std::string &filename);
			virtual ~dielTab() {}

			/// \brief Read dielectric information from a file
			/// \param filename is the name of the file to read. If blank, use _filename.
			void read(const std::string &filename = "");
			/// Read dielectric information from a stream
			void read(std::istream &in, size_t length = 0);
			/// Read dielectric information from a string
			void readString(const std::string &in);
			/// Write dielectric information to a file
			void write(const std::string &fname) const;
			/// Write dielectric information to a stream
			void write(std::ostream &out) const;

			/// Interpolate the dielectric at a frequency based on known values
			std::complex<double> interpolate(double freq) const;

			/// Description of the dielectric information
			std::string title;
			/// Mapping between frequency and known dielectric values
			std::map<double, std::complex<double> > freqMMap;
			/// Mappings between the column ordering in the dielectric file and the structure data
			size_t colMaps[dielColumns::NUMCOLS];

			/// Calculates the hash of the dielectic. Convenience function.
			HASH_t hash() const;

			

			typedef std::function<std::complex<double> (double freq) > mProvider;

			/**
			* \brief The static generator can produce a diel.tab file given 
			* a set of frequencies. 
			*
			* If the number of frequencies is less 
			* than three, then interpolation would fail and a Liu-style 
			* diel.tab file will be generated. The function also takes 
			* a functional object that provides the refractive indices. 
			* Imaginary refractive index parts are always taken as positive 
			* values per ddscat conventions.

			* If the refractive indices are already known, then the generator is 
			* not necessary, and the freqMMap can simply be populated with 
			* the known frequency pairs.
			**/
			template<class freqIter>
			static boost::shared_ptr<dielTab> 
				generate(mProvider provider, const freqIter pstart, const freqIter pend)
			{
				size_t numFreqs = 0;
				boost::shared_ptr<dielTab> tgt(new dielTab);
				while (pstart != pend)
				{
					std::complex<double> m = provider(*pstart);
					tgt->freqMMap[*pstart] = m;
					numFreqs++;
					pstart++;
				}
				if (!numFreqs) throw; // Wrong function is being used
				if (numFreqs <=3)
				{
					// Not enough values for interpolation. Take the first value in the 
					// map and duplicate it.
					std::complex<double> m = tgt->freqMMap.begin()->second;
					tgt->freqMMap.clear();
					tgt->freqMMap[0.000001] = m;
					tgt->freqMMap[1.0] = m;
					tgt->freqMMap[100000] = m;
				}
				return tgt;
			}

			/// Generates a diel.tab structure equivalent to the rtmath::refract::writeDiel function
			static boost::shared_ptr<dielTab>
				generate(const std::complex<double> &m)
			{
				boost::shared_ptr<dielTab> tgt(new dielTab);
				tgt->freqMMap.clear();
				tgt->freqMMap[0.000001] = m;
				tgt->freqMMap[1.0] = m;
				tgt->freqMMap[100000] = m;
				return tgt;
			}

		private:
			std::string _filename;
			void _init();
			bool _colMapsValid() const;
		};
	}
}
