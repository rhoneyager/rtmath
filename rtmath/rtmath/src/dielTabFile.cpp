#include "Stdafx-ddscat.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <set>
#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include "../rtmath/ddscat/dielTabFile.h"
#include "../rtmath/refract.h"
#include "../rtmath/macros.h"

#include "../rtmath/error/error.h"
namespace rtmath {
	namespace ddscat {

		void dielTab::_init()
		{
			//std::fill_n(&colMaps[0],dielColumns::NUMCOLS, 0);
			colMaps[dielColumns::WAVELENGTH] = 1;
			colMaps[dielColumns::M_RE] = 2;
			colMaps[dielColumns::M_IM] = 3;
			colMaps[dielColumns::E_RE] = 4;
			colMaps[dielColumns::E_IM] = 5;
		}

		dielTab::dielTab()
		{
			_init();
		}

		dielTab::dielTab(const std::string &filename)
		{
			_init();
			read(filename);
		}

		void dielTab::read(const std::string &filename)
		{
			using namespace std;
			using namespace boost::interprocess;
			using namespace boost::filesystem;

			string fname = filename;
			if (fname == "")
			{
				if (_filename.size())
				{
					fname = _filename;
				} else {
					throw rtmath::debug::xBadInput("Must specify filename");
				}
			}

			if (!exists(path(fname)))
				throw rtmath::debug::xMissingFile(fname.c_str());

			size_t fsize = (size_t) file_size(path(fname)); // bytes

			file_mapping m_file(
				fname.c_str(),
				read_only
				);

			mapped_region region (
				m_file,
				read_only,
				0,
				fsize);

			void* start = region.get_address();
			const char* a = (char*) start;

			_filename = fname;
			string s(a, fsize);
			readString(s);
		}

		void dielTab::read(std::istream &in, size_t length)
		{
			using namespace std;
			if (!length)
			{
				in.seekg(0, ios::end);
				length = (size_t) in.tellg();
				in.seekg(0, ios::beg);
			}

			char* sb = new char[length];
			in.read(sb,length);
			string s(sb,length);
			delete[] sb;

			readString(s);
		}

		void dielTab::readString(const std::string &in)
		{
			// Since istringstream is so slow, I'm dusting off my old atof macros (in 
			// macros.h). These were used when I implemented lbl, and are very fast.
			using namespace std;
			_init();

			size_t length = in.size();

			// First, do header processing
			size_t pend = 0;
			size_t importMap[5]; // Import column maps, straight from the file.
			{
				// Seek to the end of the header, and construct an istringstream for just the header
				size_t pstart = 0;
				for (size_t i=0; i<3; i++)
				{
					pstart = pend;
					pend = in.find_first_of("\n", pend+1);
					size_t posa = 0, posb = pstart;
					switch (i)
					{
					case 0: // Title line
						title = string(in.data(),pend);
						break;
					case 2: // Junk line
					default:
						break;
					case 1: // Column mappings
						// Read in five unsigned ints, then assign.
						{
							size_t *v = importMap; // Relic from shapefile import...
							for (size_t j=0;j<5;j++)
							{
								// Seek to first nonspace character
								posa = in.find_first_not_of(" \t\n,", posb);
								// Find first space after this position
								posb = in.find_first_of(" \t\n,", posa);
								size_t len = posb - posa;
								v[j] = (size_t) rtmath::macros::m_atoi(&(in.data()[posa]),len);
							}
						}
						break;
					}
				}
			}

			// Finish the column mappings
			if (!importMap[0]) throw rtmath::debug::xBadInput("Cannot make diel map without wavelengths.");
			// eps = m^2
			// m = mr + i mi, where mr, mi > 0
			// The easy cases:
			bool mBasic = (importMap[1] && importMap[2]);
			bool mFromE = (importMap[3] && importMap[4]);
			// The harder cases:
			bool mrer_mi = (importMap[1] && importMap[3]);
			bool mier_mr = (importMap[2] && importMap[3]);
			bool miei_mr = (importMap[2] && importMap[4]);
			bool mrei_mi = (importMap[1] && importMap[4]);

			bool sufficient = (mBasic || mFromE || ( (mier_mr || miei_mr) && ( mrer_mi || mrei_mi) ) );
			if (!sufficient) throw rtmath::debug::xBadInput("Cannot make diel map with incomplete information.");


			size_t posa = 0, posb = pend+1;
			size_t i=0;
			// Load in the dielectrics through iteration and macro.h-based double extraction
			for (;; i++) // Loop through to end of file, while keeping count of number of dielectrics
			{
				vector<double> valser(6); // valser(0) will remain zero - it's a dummy entry for ease of programming
				for (size_t j=1; j<6; j++) // Do not support lines with text.
				{
					// Seek to first nonspace character
					posa = in.find_first_not_of(" \t\n", posb);
					if (posa == string::npos) break; // At end of retreival
					// Find first space after this position
					posb = in.find_first_of(" \t\n", posa);
					if (posb == string::npos) // End of file condition
					{
						// Try to get one last value
						posb = in.find_last_of("0123456789", posa);
						if (posb == string::npos) break; // Value retreival impossible
					}
					size_t len = posb - posa;
					double val;
					val = rtmath::macros::m_atof(&(in.data()[posa]),len);
					valser[j] = val;
				}

				double wavelength = valser[importMap[0]]; // Wavelength is guaranteed at this point
				// The rest are not
				// TODO: Consider doing this using a depGraph-variant...
				double mr = valser[importMap[1]];
				double mi = valser[importMap[2]];
				double er = valser[importMap[3]];
				double ei = valser[importMap[4]];
				if (mBasic)
				{}
				else if (mFromE)
				{
					complex<double> m = sqrt(complex<double>(er,ei));
					mr = m.real();
					mi = m.imag();
				} else {
					if (mier_mr) mr = sqrt(mi*mi-er);
					else if (miei_mr) mr = ei / (2. * mi);
					if (mrer_mi) mi = sqrt(mr*mr-er);
					else if (mrei_mi) mi = ei / (2.*mr);
				}

				// m is known by this point
				complex<double> m(mr,mi);
				freqMMap[wavelength] = m;
			}
		}

		bool dielTab::_colMapsValid() const
		{
			// Column maps can be zero through five for writing.
			// Aside from zero, the maps cannot coincide.
			using namespace std;
			set<size_t> occupied;

			for (size_t it = 0; it < (size_t) dielColumns::NUMCOLS; ++it)
			{
				size_t col = colMaps[it];
				if (col)
				{
					if (col > 5) return false;
					if (occupied.count(col)) return false;
					occupied.insert(col);
				}
			}

			// Check that the column outputs are not disjoint
			// So, check rbegin, and its value should equal size
			{
				auto it = occupied.rbegin();
				size_t max = *it;
				if (max != occupied.size()) return false;
			}

			return true;
		}

		void dielTab::write(std::ostream &out) const
		{
			using namespace std;

			if (!freqMMap.size()) throw rtmath::debug::xArrayOutOfBounds();
			if (!_colMapsValid()) throw rtmath::debug::xBadInput("Bad diel column mappings");
			out.setf( ios::scientific, ios::floatfield);
			out.precision(7);

			//out << " m = " << ref.real() << " + " << (-1.0 *ref.imag()) << " i" << endl;
			out << title;
			if (!title.size())
			{
				out << "Autogenerated dielectrics starting with m = " 
					<< freqMMap.begin()->second.real() << " + " 
					<< abs(freqMMap.begin()->second.imag()) << " i";
			}
			out << endl;

			out << " " << colMaps[dielColumns::WAVELENGTH] << " " 
				<< colMaps[dielColumns::M_RE] << " " << colMaps[dielColumns::M_IM] << 
				" " << colMaps[dielColumns::E_RE] << " " << colMaps[dielColumns::E_IM];
			out << " = columns for wave, Re(m), Im(m), eps1, eps2" << endl;

			// Write out data in the decided order. Also produce a header that 
			// reflects this choice.

			// Produce the reverse map. Original map is guaranteed to not be disjoint.
			size_t rmap[dielColumns::NUMCOLS];
			for (size_t i=0; i< dielColumns::NUMCOLS; ++i)
				rmap[i] = 0;
			//std::fill_n(rmap,dielColumns::NUMCOLS, 0);
			for (size_t i=0; i< dielColumns::NUMCOLS; ++i)
			{
				if (size_t loc = colMaps[i] )
					rmap[loc] = i;
			}
			// The reverse map lists the columns in order, and is padded with zeros 
			// at the end.

			// Write the relevant column names
			//out << " LAMBDA       Re(N)         Im(N)" << endl;
			out << " ";
			for (size_t i=0; i<dielColumns::NUMCOLS; ++i)
//			for (size_t i=0; i<dielColumns::NUMCOLS, rmap[i]; ++i)
			{
				if (i) out << "\t\t";
				if (i == dielColumns::WAVELENGTH) out << "LAMBDA";
				if (i == dielColumns::M_RE) out << "Re(M)";
				if (i == dielColumns::M_IM) out << "Im(M)";
				if (i == dielColumns::E_RE) out << "Re(E)";
				if (i == dielColumns::E_IM) out << "Im(E)";
			}
			//out << endl;

			// Write the relevant values
			//out << " 0.000001    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
			//out << " 1.000000    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
			//out << " 100000.0    " << ref.real() << "      " << (-1.0*ref.imag()) << endl;
			auto freqMMap = this->freqMMap; // Override the class variable in case additional values need to be added
			if (freqMMap.size() < 3)
			{
				std::complex<double> m = freqMMap.begin()->second;
				freqMMap[0.000001] = m;
				freqMMap[1] = m;
				freqMMap[100000] = m;
			}

			for (auto it = freqMMap.begin(); it != freqMMap.end(); ++it)
			{
				//out << " " << it->first << "    " << it->second.real() 
				//	<< "      " << abs(it->second.imag()) << endl;
				out << "\n ";
				complex<double> m = it->second;
				complex<double> e;
				rtmath::refract::mToE(m, e);
				for (size_t i=0; i<dielColumns::NUMCOLS; ++i)
				{
					if (i) out << "\t";
					if (i == dielColumns::WAVELENGTH) out << it->first;
					if (i == dielColumns::M_RE) out << m.real();
					if (i == dielColumns::M_IM) out << abs(m.imag());
					if (i == dielColumns::E_RE) out << e.real();
					if (i == dielColumns::E_IM) out << abs(e.imag());
				}
			}
		}

		void dielTab::write(const std::string &filename) const
		{
			using namespace std;
			ofstream out(filename.c_str());
			write(out);
		}

		std::complex<double> dielTab::interpolate(double freq) const
		{
			using namespace std;
			complex<double> res;
			if (freqMMap.size() == 0) throw debug::xArrayOutOfBounds();
			// Perform linear interpolation based on known dielectric values.
			// If only one dielectric value is present, just return it.
			auto it = freqMMap.begin();
			if (freqMMap.size() == 1) return it->second;
			if ((it = freqMMap.find(freq)) != freqMMap.end()) return it->second;
			it = freqMMap.upper_bound(freq); // Key found is greater. Can be at beginning.
			if (it == freqMMap.begin())
			{
				// Guaranteed to have at least two elements
				auto ot = it;
				ot++;
				complex<double> rise = ot->second - it->second;
				complex<double> run(ot->first - it->first,0);
				auto slope = rise / run;
				res = slope * complex<double>(freq - it->first,0) + it->second;
			} else if (it == freqMMap.end()) {
				auto ot = it;
				ot--;
				auto pt = ot;
				pt--;
				complex<double> rise = ot->second - pt->second;
				complex<double> run(ot->first - pt->first,0);
				auto slope = rise / run;
				res = slope * complex<double>(freq - ot->first,0) + ot->second;
			} else {
				auto ot = it;
				ot--;
				// ot is below, it is above
				complex<double> rise = it->second - ot->second;
				complex<double> run(it->first - ot->first,0);
				auto slope = rise / run;
				res = slope * complex<double>(freq - it->first,0) + it->second;
			}
			return res;
		}

	}
}
