#include "Stdafx-ddscat_base.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/algorithm/string.hpp>
#include <boost/iostreams/filter/newline.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#if USE_RYAN_SERIALIZATION
#include <Ryan_Serialization/serialization.h>
#endif
#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/config.h"
#include "../rtmath/splitSet.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {

		namespace ddParParsers
		{

			bool ddParLine::operator==(ddParLine &rhs)
			{
				if (_id != rhs._id) return false;
				if (_endWriteWithEndl != rhs._endWriteWithEndl) return false;
				std::ostringstream oT, oR;
				write(oT, ddVersions::getDefaultVer());
				rhs.write(oR, ddVersions::getDefaultVer());
				std::string sT = oT.str(), sR = oR.str();
				if (sT != sR) return false;

				return true;
			}

			bool ddParLine::operator!=(ddParLine &rhs)
			{
				return !(operator==(rhs));
			}

			bool ddParLine::operator<(ddParLine &rhs)
			{
				if (_id != rhs._id) return (_id < rhs._id);
				std::ostringstream oT, oR;
				write(oT, ddVersions::getDefaultVer());
				rhs.write(oR, ddVersions::getDefaultVer());
				std::string sT = oT.str(), sR = oR.str();
				return (sT < sR);
			}

		}

		// ddPar overrides
		// These are template functions (for int, double, size_t) that implement the 
		// easy-use interface
		template<class T>
		T ddPar::__getSimple(ddParParsers::ParId key) const
		{
			boost::shared_ptr<const ddParParsers::ddParLineSimple<T> > line;
			boost::shared_ptr< const ddParParsers::ddParLine > linein;
			T v;
			getKey(key, linein);
			line = boost::static_pointer_cast< const ddParParsers::ddParLineSimple<T> >(linein);
			line->get(v);
			return v;
		}

		template size_t ddPar::__getSimple<size_t>(ddParParsers::ParId) const;
		template bool ddPar::__getSimple<bool>(ddParParsers::ParId) const;
		template double ddPar::__getSimple<double>(ddParParsers::ParId) const;
		template int ddPar::__getSimple<int>(ddParParsers::ParId) const;

		template<class T>
		void ddPar::__setSimple(ddParParsers::ParId key, T val)
		{
			boost::shared_ptr< ddParParsers::ddParLineSimple<T> >
				line(new ddParParsers::ddParLineSimple<T>(key));
			line->set(val);
			insertKey(key, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		template void ddPar::__setSimple<size_t>(ddParParsers::ParId, size_t);
		template void ddPar::__setSimple<bool>(ddParParsers::ParId, bool);
		template void ddPar::__setSimple<double>(ddParParsers::ParId, double);
		template void ddPar::__setSimple<int>(ddParParsers::ParId, int);

		bool ddPar::__getSimpleBool(ddParParsers::ParId key) const
		{
			boost::shared_ptr<const ddParParsers::ddParLineSimple<size_t> > line;
			boost::shared_ptr< const ddParParsers::ddParLine > linein;
			size_t v;
			getKey(key, linein);
			line = boost::static_pointer_cast< const ddParParsers::ddParLineSimple<size_t> >(linein);
			line->get(v);
			return (v) ? true : false;
		}

		void ddPar::__setSimpleBool(ddParParsers::ParId key, bool val)
		{
			boost::shared_ptr< ddParParsers::ddParLineSimple<size_t> >
				line(new ddParParsers::ddParLineSimple<size_t>(key));
			size_t vi = (val) ? 1 : 0;
			line->set(vi);
			insertKey(key, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		template<class valtype>
		valtype ddPar::__getSimplePlural(ddParParsers::ParId id, size_t index) const
		{
			boost::shared_ptr<const ddParParsers::ddParLineSimplePlural<valtype> > line;
			boost::shared_ptr< const ddParParsers::ddParLine > linein;
			valtype v;
			getKey(id, linein);
			line = boost::static_pointer_cast< const ddParParsers::ddParLineSimplePlural<valtype> >(linein);
			line->get(index, v);
			return v;
		}

		template size_t ddPar::__getSimplePlural<size_t>(ddParParsers::ParId, size_t) const;
		template bool ddPar::__getSimplePlural<bool>(ddParParsers::ParId, size_t) const;
		template double ddPar::__getSimplePlural<double>(ddParParsers::ParId, size_t) const;
		template int ddPar::__getSimplePlural<int>(ddParParsers::ParId, size_t) const;

		template<class valtype>
		void ddPar::__setSimplePlural(ddParParsers::ParId id, size_t index, size_t maxSize, const valtype &v)
		{
			boost::shared_ptr< ddParParsers::ddParLineSimplePlural<valtype> > line;
			boost::shared_ptr< ddParParsers::ddParLine > linein;
			// does key exist? if so, load it and prepare for update
			if (exists(id))
			{
				getKey(id, linein);
				line = boost::static_pointer_cast< ddParParsers::ddParLineSimplePlural<valtype> >(linein);
			}
			else {
				line = boost::shared_ptr<ddParParsers::ddParLineSimplePlural<valtype> >
					(new ddParParsers::ddParLineSimplePlural<valtype>(id));
				line->resize(maxSize);
			}
			line->set(index, v);
			insertKey(id, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		template void ddPar::__setSimplePlural<size_t>(ddParParsers::ParId, size_t, size_t, const size_t&);
		template void ddPar::__setSimplePlural<bool>(ddParParsers::ParId, size_t, size_t, const bool&);
		template void ddPar::__setSimplePlural<double>(ddParParsers::ParId, size_t, size_t, const double&);
		template void ddPar::__setSimplePlural<int>(ddParParsers::ParId, size_t, size_t, const int&);

		template<class valtype>
		void ddPar::__setSimplePluralTuple(ddParParsers::ParId id, size_t index, size_t maxSize, const valtype &v, size_t tuplesz)
		{
			boost::shared_ptr< ddParParsers::ddParTuples<valtype> > line;
			boost::shared_ptr< ddParParsers::ddParLine > linein;
			// does key exist? if so, load it and prepare for update
			if (exists(id))
			{
				getKey(id, linein);
				line = boost::static_pointer_cast< ddParParsers::ddParTuples<valtype> >(linein);
			}
			else {
				line = boost::shared_ptr<ddParParsers::ddParTuples<valtype> >
					(new ddParParsers::ddParTuples<valtype>(tuplesz, id));
				line->resize(maxSize);
			}
			line->set(index, v);
			insertKey(id, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		template void ddPar::__setSimplePluralTuple<size_t>(ddParParsers::ParId, size_t, size_t, const size_t&, size_t);
		template void ddPar::__setSimplePluralTuple<bool>(ddParParsers::ParId, size_t, size_t, const bool&, size_t);
		template void ddPar::__setSimplePluralTuple<double>(ddParParsers::ParId, size_t, size_t, const double&, size_t);
		template void ddPar::__setSimplePluralTuple<int>(ddParParsers::ParId, size_t, size_t, const int&, size_t);

		void ddPar::__getString(ddParParsers::ParId id, std::string &val) const
		{
			boost::shared_ptr<const ddParParsers::ddParLineSimple<std::string> > line;
			boost::shared_ptr< const ddParParsers::ddParLine > linein;
			std::string v;
			getKey(id, linein);
			line = boost::static_pointer_cast< const ddParParsers::ddParLineSimple<std::string> >(linein);
			line->get(v);
			val = v;
		}

		void ddPar::__setString(ddParParsers::ParId id, const std::string &val)
		{
			boost::shared_ptr< ddParParsers::ddParLineSimple<std::string> >
				line(new ddParParsers::ddParLineSimple<std::string>(id));
			line->set(val);
			insertKey(id, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		bool ddPar::__getStringBool(ddParParsers::ParId id, const std::string &bfalse, const std::string &btrue) const
		{
			boost::shared_ptr<const ddParParsers::ddParLineSimple<std::string> > line;
			boost::shared_ptr< const ddParParsers::ddParLine > linein;
			std::string v;
			getKey(id, linein);
			line = boost::static_pointer_cast< const ddParParsers::ddParLineSimple<std::string> >(linein);
			line->get(v);
			if (v == btrue)
				return true;
			return false;
		}

		void ddPar::__setStringBool(ddParParsers::ParId id, bool v, const std::string &bfalse, const std::string &btrue)
		{
			boost::shared_ptr<ddParParsers::ddParLineSimple<std::string> >
				line(new ddParParsers::ddParLineSimple<std::string>(id));
			std::string vs = (v) ? btrue : bfalse;
			line->set(vs);
			insertKey(id, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}


		namespace ddParParsers
		{
			boost::shared_ptr<ddParLine> mapKeys(const std::string &key)
			{
				using namespace std;
				using namespace rtmath::ddscat::ddParParsers;
				boost::shared_ptr<ddParLine> ptr;

				if (key.find("CMTORQ") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::string> >
					(new ddParLineSimple<std::string>(CMTORQ));
				else if (key.find("CMDSOL") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::string> >
					(new ddParLineSimple<std::string>(CMDSOL));
				else if (key.find("FFT") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::string> >
					(new ddParLineSimple<std::string>(CMDFFT));
				else if (key.find("CALPHA") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::string> >
					(new ddParLineSimple<std::string>(CALPHA));
				else if (key.find("CBINFLAG") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::string> >
					(new ddParLineSimple<std::string>(CBINFLAG));
				else if (key.find("dimension") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimplePlural<size_t> >
					(new ddParLineSimplePlural<size_t>(DIMENSION));
				else if (key.find("CSHAPE") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::string> >
					(new ddParLineSimple<std::string>(CSHAPE));
				else if (key.find("shape parameters") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimplePlural<double> >
					(new ddParLineSimplePlural<double>(SHAPEPARAMS));
				else if (key.find("NCOMP") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::size_t> >
					(new ddParLineSimple<std::size_t>(NCOMP));
				else if (key.find("refractive index") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::string> >
					(new ddParLineSimple<std::string>(IREFR));
				else if (key.find("NRFLD") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::size_t> >
					(new ddParLineSimple<std::size_t>(NRFLD));
				// version 7.2 NRFLD
				else if (key.find("extens.") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimplePlural<double> >
					(new ddParLineSimplePlural<double>(FRACT_EXTENS));
				// version 7.2 FRACT_EXTENS
				else if (key.find("TOL") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<double> >
					(new ddParLineSimple<double>(TOL));
				else if (key.find("MXITER") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::size_t> >
					(new ddParLineSimple<std::size_t>(MXITER));
				// version 7.2 MXITER
				else if (key.find("GAMMA") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<double> >
					(new ddParLineSimple<double>(GAMMA));
				else if (key.find("ETASCA") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<double> >
					(new ddParLineSimple<double>(ETASCA));

				/// \todo fix wavelengths and aeff to read two doubles, a size_t and a string
				else if (key.find("wavelengths") != string::npos)
					ptr = boost::shared_ptr<ddParLineMixed<double, std::string> >
					(new ddParLineMixed<double, std::string>(3, 4, WAVELENGTHS));
				else if (key.find("NAMBIENT") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<double> >
					(new ddParLineSimple<double>(NAMBIENT));
				// version 7.0 NAMBIENT
				else if (key.find("eff") != string::npos)
					ptr = boost::shared_ptr<ddParLineMixed<double, std::string> >
					(new ddParLineMixed<double, std::string>(3, 4, AEFF));

				else if (key.find("Polarization state") != string::npos)
					ptr = boost::shared_ptr<ddParTuples<double> >
					(new ddParTuples<double>(2, POLSTATE));
				else if (key.find("IORTH") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::size_t> >
					(new ddParLineSimple<std::size_t>(IORTH));
				else if (key.find("IWRKSC") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::size_t> >
					(new ddParLineSimple<std::size_t>(IWRKSC));
				else if (key.find("IWRPOL") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::size_t> >
					(new ddParLineSimple<std::size_t>(IWRPOL));
				// IWRPOL is version 7.0

				else if (key.find("NBETA") != string::npos)
					ptr = boost::shared_ptr<ddParLineMixed<double, size_t> >
					(new ddParLineMixed<double, size_t>(2, 3, NBETA));
				else if (key.find("NTHETA") != string::npos)
					ptr = boost::shared_ptr<ddParLineMixed<double, size_t> >
					(new ddParLineMixed<double, size_t>(2, 3, NTHETA));
				else if (key.find("NPHI") != string::npos)
					ptr = boost::shared_ptr<ddParLineMixed<double, size_t> >
					(new ddParLineMixed<double, size_t>(2, 3, NPHI));
				else if (key.find("IWAV") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimplePlural<int> >
					(new ddParLineSimplePlural<int>(IWAV));
				else if (key.find("NSMELTS") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::size_t> >
					(new ddParLineSimple<std::size_t>(NSMELTS));
				else if (key.find("indices ij of") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimplePlural<std::size_t> >
					(new ddParLineSimplePlural<std::size_t>(INDICESIJ));
				else if (key.find("CMDFRM") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::string> >
					(new ddParLineSimple<std::string>(CMDFRM));
				else if (key.find("NPLANES") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimple<std::size_t> >
					(new ddParLineSimple<std::size_t>(NPLANES));
				else if (key.find("for plane") != string::npos)
					ptr = boost::shared_ptr<ddParLineSimplePlural<double> >
					(new ddParLineSimplePlural<double>(PLANE1));
				else
				{
					//cerr << "Unmatched key: " << key << endl;
					RTthrow(rtmath::debug::xBadInput())
					<< rtmath::debug::default_file_name("ddscat.par")
					<< rtmath::debug::otherErrorText("Unmatched key")
					<< rtmath::debug::key(key);
				}

				return ptr;
			}

			bool ddParLine::versionValid(size_t ver) const
			{
				if (rtmath::ddscat::ddVersions::isVerWithin(ver, 72, 0))
				{
					switch (_id)
					{
					case IWRPOL:
						return false;
					default:
						return true;
					}
				}
				if (rtmath::ddscat::ddVersions::isVerWithin(ver, 0, 72)) // up through 7.1 (inclusive)
				{
					switch (_id)
					{
					case NRFLD:
					case FRACT_EXTENS:
					case MXITER:
					case NAMBIENT:
						return false;
					default:
						return true;
					}
				}
				return true;
			}

			bool commentString(ParId id, std::string &key, size_t version)
			{
				switch (id)
				{
				case CMTORQ:
					key = "'**** Preliminaries ****'";
					break;
				case DIMENSION:
					key = "'**** Initial Memory Allocation ****'";
					break;
				case CSHAPE:
					key = "'**** Target Geometry and Composition ****'";
					break;
				case NRFLD:
					key = "'**** Additional Nearfield calculation? ****'";
					break;
				case TOL:
					key = "'**** Error Tolerance ****'";
					break;
				case MXITER:
					if (ddVersions::isVerWithin(version, 0, 73))
					{
						key = "'**** maximum number of iterations allowed ****'";
					}
					else {
						key = "'**** Maximum number of iterations ****'";
					}
					break;
				case GAMMA:
					if (ddVersions::isVerWithin(version, 0, 73))
					{
						key = "'**** Interaction cutoff parameter for PBC calculations ****'";
					}
					else {
						key = "'**** Integration cutoff parameter for PBC calculations ****'";
					}
					break;
				case ETASCA:
					key = "'**** Angular resolution for calculation of <cos>, etc. ****'";
					break;
				case WAVELENGTHS:
					if (ddVersions::isVerWithin(version, 0, 73))
					{
						key = "'**** Wavelengths (micron) ****'";
					}
					else {
						key = "'**** Vacuum wavelengths (micron) ****'";
					}
					break;
				case NAMBIENT:
					key = "'**** Refractive index of ambient medium'";
					break;
				case AEFF:
					key = "'**** Effective Radii (micron) **** '";
					break;
				case POLSTATE:
					key = "'**** Define Incident Polarizations ****'";
					break;
				case IWRKSC:
					key = "'**** Specify which output files to write ****'";
					break;
				case NBETA:
					if (ddVersions::isVerWithin(version, 0, 73))
					{
						key = "'**** Prescribe Target Rotations ****'";
					}
					else {
						key = "'**** Specify Target Rotations ****'";
					}
					break;
				case IWAV:
					key = "'**** Specify first IWAV, IRAD, IORI (normally 0 0 0) ****'";
					break;
				case NSMELTS:
					key = "'**** Select Elements of S_ij Matrix to Print ****'";
					break;
				case CMDFRM:
					key = "'**** Specify Scattered Directions ****'";
					break;
				default:
					return false;
				}
				return true;
			}

			void pString(const std::string &in, std::string &out)
			{
				out = in;
				// Remove all single quotes from the string
				boost::algorithm::erase_all(out, "\'");
			}

			void idString(ParId id, std::string &key, size_t version)
			{
				switch (id)
				{
				case CMTORQ:
					key = "CMTORQ*6 (NOTORQ, DOTORQ) -- either do or skip torque calculations";
					break;
				case CMDSOL:
					key = "CMDSOL*6 (PBCGS2, PBCGST, PETRKP) -- select solution method";
					break;
				case CMDFFT:
					if (ddVersions::isVerWithin(version, 0, 73))
					{
						key = "CMDFFT*6 (GPFAFT, FFTMKL) --- FFT method";
					}
					else {
						key = "CMETHD*6 (GPFAFT, FFTMKL) -- FFT method";
					}
					break;
				case CALPHA:
					key = "CALPHA*6 (GKDLDR, LATTDR)";
					break;
				case CBINFLAG:
					key = "CBINFLAG (NOTBIN, ORIBIN, ALLBIN)";
					break;
				case DIMENSION:
					key = "dimension";
					break;
				case CSHAPE:
					key = "CSHAPE*9 shape directive";
					break;
				case SHAPEPARAMS:
					key = "shape parameters 1-3";
					break;
				case NCOMP:
					key = "NCOMP = number of dielectric materials";
					break;
				case IREFR:
					key = "file with refractive index";
					break;
				case NRFLD:
					key = "NRFLD (=0 to skip nearfield calc., =1 to calculate nearfield E)";
					break;
				case FRACT_EXTENS:
					key = "(fract. extens. of calc. vol. in -x,+x,-y,+y,-z,+z)";
					break;
				case TOL:
					key = "TOL = MAX ALLOWED (NORM OF |G>=AC|E>-ACA|X>)/(NORM OF AC|E>)";
					break;
				case MXITER:
					key = "MXITER";
					break;
				case GAMMA:
					key = "GAMMA (1e-2 is normal, 3e-3 for greater accuracy)";
					break;
				case ETASCA:
					key = "ETASCA (number of angles is proportional to [(3+x)/ETASCA]^2 )";
					break;
				case WAVELENGTHS:
					if (ddVersions::isVerWithin(version, 0, 73))
					{
						key = "wavelengths";
					}
					else {
						key = "wavelengths (first,last,how many,how=LIN,INV,LOG)";
					}
					break;
				case NAMBIENT:
					key = "NAMBIENT";
					break;
				case AEFF:
					if (ddVersions::isVerWithin(version, 0, 73))
					{
						key = "aeff";
					}
					else {
						key = "eff. radii (first, last, how many, how=LIN,INV,LOG)";
					}
					break;
				case POLSTATE:
					key = "Polarization state e01 (k along x axis)";
					break;
				case IORTH:
					key = "IORTH  (=1 to do only pol. state e01; =2 to also do orth. pol. state)";
					break;
				case IWRKSC:
					key = "IWRKSC (=0 to suppress, =1 to write \".sca\" file for each target orient.";
					break;
				case IWRPOL:
					key = "IWRPOL (=0 to suppress, =1 to write \".pol\" file for each (BETA,THETA)";
					break;
				case NBETA:
					key = "BETAMI, BETAMX, NBETA  (beta=rotation around a1)";
					break;
				case NTHETA:
					key = "THETMI, THETMX, NTHETA (theta=angle between a1 and k)";
					break;
				case NPHI:
					key = "PHIMIN, PHIMAX, NPHI (phi=rotation angle of a1 around k)";
					break;
				case IWAV:
					key = "first IWAV, first IRAD, first IORI (0 0 0 to begin fresh)";
					break;
				case NSMELTS:
					key = "NSMELTS = number of elements of S_ij to print (not more than 9)";
					break;
				case INDICESIJ:
					key = "indices ij of elements to print";
					break;
				case CMDFRM:
					key = "CMDFRM (LFRAME, TFRAME for Lab Frame or Target Frame)";
					break;
				case NPLANES:
					key = "NPLANES = number of scattering planes";
					break;
				case PLANE1:
					key = "phi, thetan_min, thetan_max, dtheta (in deg) for plane";
					break;
				case UNKNOWN:
				default:
					RTthrow(rtmath::debug::xBadInput())
					<< rtmath::debug::otherErrorText("Unknown parid");
				}
			}
		} // end ddparparsers
	} // end namespace ddscat
} // end rtmath


