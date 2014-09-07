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

		bool ddPar::exists(ddParParsers::ParId key) const
		{
			if (_parsedData.count(key)) return true;
			return false;
		}

		void ddPar::getKey(ddParParsers::ParId key, boost::shared_ptr<ddParParsers::ddParLine> &res)
		{
			if (_parsedData.count(key))
				res = _parsedData[key];
		}

		void ddPar::getKey(ddParParsers::ParId key, boost::shared_ptr<const ddParParsers::ddParLine> &res) const
		{
			if (_parsedData.count(key))
				res = _parsedData[key];
		}

		void ddPar::delKey(ddParParsers::ParId key)
		{
			if (_parsedData.count(key))
				_parsedData.erase(key);
		}

		void ddPar::insertKey(ddParParsers::ParId key, boost::shared_ptr<ddParParsers::ddParLine> ptr)
		{
			if (_parsedData.count(key))
				_parsedData.erase(key);
			_parsedData[key] = ptr;
		}

		void ddPar::getSIJ(std::set<size_t> &sij) const
		{
			sij.clear();

			boost::shared_ptr< const ddParParsers::ddParLineSimplePlural<size_t> > line;
			boost::shared_ptr< const ddParParsers::ddParLine > linein;
			std::vector<size_t> val;
			// NSMELTS, INDICESIJ
			getKey(ddParParsers::INDICESIJ, linein);
			line = boost::static_pointer_cast< const ddParParsers::ddParLineSimplePlural<size_t> >(linein);
			line->get(val);
			for (auto it = val.begin(); it != val.end(); it++)
			{
				if (!sij.count(*it))
					sij.insert(*it);
			}
		}

		void ddPar::setSIJ(const std::set<size_t> &sij)
		{
			// Set array size NSMELTS, with max of ?
			boost::shared_ptr<ddParParsers::ddParLineSimple<std::size_t> > line_size
				(new ddParParsers::ddParLineSimple<std::size_t>(ddParParsers::NSMELTS));
			line_size->set(sij.size());

			// Set array
			boost::shared_ptr< ddParParsers::ddParLineSimplePlural<size_t> > line
				(new ddParParsers::ddParLineSimplePlural<size_t>(ddParParsers::INDICESIJ));
			std::vector<size_t> val;
			for (auto it = sij.begin(); it != sij.end(); it++)
				val.push_back(*it);
			line->set(val);

			insertKey(ddParParsers::NSMELTS, boost::static_pointer_cast< ddParParsers::ddParLine >(line_size));
			insertKey(ddParParsers::INDICESIJ, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		void ddPar::getAeff(double &min, double &max, size_t &n, std::string &spacing) const
		{
			boost::shared_ptr< const ddParParsers::ddParLineMixed<double, std::string> > line;

			boost::shared_ptr< const ddParParsers::ddParLine > linein;
			getKey(ddParParsers::AEFF, linein);
			line = boost::static_pointer_cast< const ddParParsers::ddParLineMixed<double, std::string> >(linein);
			line->setSep(3, 4);
			line->get<double>(0, min);
			line->get<double>(1, max);
			double dN;
			line->get<double>(2, dN);
			n = (size_t)dN;

			line->get<std::string>(3, spacing);
		}

		std::string ddPar::getAeff() const
		{
			/// \todo Add in tab file reading
			double min, max;
			size_t n;
			std::string spacing, res;
			getAeff(min, max, n, spacing);
			std::ostringstream out;
			out << min << ":" << n << ":" << max << ":" << spacing;
			res = out.str();
			return res;
		}

		/// Labels the typical polarization states. Not rigorous by any means.
		std::string ddPar::namePolState() const
		{
			const double pols[6] = { PolState(0), PolState(1), PolState(2), 
				PolState(3), PolState(4), PolState(5) };
			auto nearPol = [](const double val[6], const double ref[6]) -> bool
			{
				// Normalize val and ref
				double normvalsq = 0, normrefsq = 0;
				for (size_t i = 0; i < 6; ++i)
				{
					normvalsq += val[i] * val[i];
					normrefsq += ref[i] * ref[i];
				}
				double normsq = 0;
				for (size_t i = 0; i < 6; ++i)
					normsq += pow((val[i] /sqrt(normvalsq))- (ref[i]/sqrt(normrefsq)), 2.);
				if (normsq < 0.01) return true;
				return false;
			};

			const double lin[6] = { 0, 0, 1, 0, 0, 0 };
			const double rhc[6] = { 0, 0, 1, 0, 0, 1 };
			const double lhc[6] = { 0, 0, 0, 1, 1, 0 };
			if (nearPol(pols, lin)) return std::string("lin");
			if (nearPol(pols, rhc)) return std::string("rhc");
			if (nearPol(pols, lhc)) return std::string("lhc");
			return std::string("unlabeled");
		}

		void ddPar::getAeff(std::set<double> &aeffs) const
		{
			std::string in = getAeff();
			rtmath::config::splitSet<double>(in, aeffs);
		}

		void ddPar::getWavelengths(std::set<double> &wvs) const
		{
			std::string in = getWavelengths();
			rtmath::config::splitSet<double>(in, wvs);
		}

		std::string ddPar::getWavelengths() const
		{
			/// \todo Add in tab file reading
			double min, max;
			size_t n;
			std::string spacing, res;
			getWavelengths(min, max, n, spacing);
			std::ostringstream out;
			out << min << ":" << n << ":" << max << ":" << spacing;
			res = out.str();
			return res;
		}

		void ddPar::getWavelengths(double &min, double &max, size_t &n, std::string &spacing) const
		{
			boost::shared_ptr< const ddParParsers::ddParLineMixed<double, std::string> > line;

			boost::shared_ptr< const ddParParsers::ddParLine > linein;
			getKey(ddParParsers::WAVELENGTHS, linein);
			line = boost::static_pointer_cast< const ddParParsers::ddParLineMixed<double, std::string> >(linein);
			line->setSep(3, 4);
			line->get<double>(0, min);
			line->get<double>(1, max);
			double dN;
			line->get<double>(2, dN);
			n = (size_t)dN;

			line->get<std::string>(3, spacing);
		}

		void ddPar::setAeff(double min, double max, size_t n, const std::string &spacing)
		{
			boost::shared_ptr< ddParParsers::ddParLineMixed<double, std::string> > line
				(new ddParParsers::ddParLineMixed<double, std::string>(3, 4, ddParParsers::AEFF));
			line->set<double>(0, min);
			line->set<double>(1, max);
			line->set<double>(2, (double)n);
			line->set<std::string>(3, spacing);
			insertKey(ddParParsers::AEFF, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		void ddPar::setWavelengths(double min, double max, size_t n, const std::string &spacing)
		{
			boost::shared_ptr< ddParParsers::ddParLineMixed<double, std::string> > line
				(new ddParParsers::ddParLineMixed<double, std::string>(3, 4, ddParParsers::WAVELENGTHS));
			line->set<double>(0, min);
			line->set<double>(1, max);
			line->set<double>(2, (double)n);
			line->set<std::string>(3, spacing);
			insertKey(ddParParsers::WAVELENGTHS, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		void ddPar::getRots(rotations &rots) const
		{
			// The rotations constructor already contains the necessary stuff. Just alias it.
			rots = rotations(*this);
		}

		void ddPar::setRots(const rotations &rots)
		{
			// use rotations code for duplication prevention
			rots.out(*this);
			/*
			boost::shared_ptr< ddParParsers::ddParLineMixed<double, size_t> > bline
			(new ddParParsers::ddParLineMixed<double, size_t>(2, ddParParsers::NBETA));
			bline->set<double>(0, rots.bMin());
			bline->set<double>(1, rots.bMax());
			bline->set<double>(2, rots.bN());


			boost::shared_ptr< ddParParsers::ddParLineMixed<double, size_t> > tline
			(new ddParParsers::ddParLineMixed<double, size_t>(2, ddParParsers::NTHETA));
			tline->set<double>(0, rots.tMin());
			tline->set<double>(1, rots.tMax());
			tline->set<double>(2, rots.tN());

			boost::shared_ptr< ddParParsers::ddParLineMixed<double, size_t> > pline
			(new ddParParsers::ddParLineMixed<double, size_t>(2, ddParParsers::NPHI));
			pline->set<double>(0, rots.pMin());
			pline->set<double>(1, rots.pMax());
			pline->set<double>(2, rots.pN());
			*/
		}

		void ddPar::getPlane(size_t key, boost::shared_ptr<ddParParsers::ddParLineSimplePlural<double> > &res)
		{
			if (_scaPlanes.count(key))
				res = _scaPlanes[key];
		}

		void ddPar::getPlane(size_t key, boost::shared_ptr<const ddParParsers::ddParLineSimplePlural<double> > &res) const
		{
			if (_scaPlanes.count(key))
				res = _scaPlanes[key];
		}

		void ddPar::getPlane(size_t n, double &phi, double &thetan_min, double &thetan_max, double &dtheta) const
		{
			boost::shared_ptr<const ddParParsers::ddParLineSimplePlural<double> > res;
			getPlane(n, res);
			res->get(0, phi);
			res->get(1, thetan_min);
			res->get(2, thetan_max);
			res->get(3, dtheta);
		}

		void ddPar::delPlane(size_t key)
		{
			if (_scaPlanes.count(key))
				_scaPlanes.erase(key);
		}

		void ddPar::insertPlane(size_t key, boost::shared_ptr<ddParParsers::ddParLineSimplePlural<double> > &res)
		{
			if (_scaPlanes.count(key))
				_scaPlanes.erase(key);
			_scaPlanes[key] = res;
		}

		void ddPar::setPlane(size_t n, double phi, double thetan_min, double thetan_max, double dtheta)
		{
			boost::shared_ptr<ddParParsers::ddParLineSimplePlural<double> > res
				(new ddParParsers::ddParLineSimplePlural<double>(ddParParsers::PLANE1));
			res->resize(4);
			res->set(0, phi);
			res->set(1, thetan_min);
			res->set(2, thetan_max);
			res->set(3, dtheta);

			insertPlane(n, res);
		}

		void ddPar::getDiels(std::vector<std::string>& res) const
		{
			//boost::shared_ptr<const ddParParsers::ddParLineSimple<std::string> > res;
			for (auto &diel : _diels)
			{
				std::string val;
				diel->get(val);
				res.push_back(val);
			}
		}

		void ddPar::getDielHashes(std::vector<HASH_t>& res) const
		{
			res = _dielHashes;
		}

		void ddPar::setDielHashes(std::vector<HASH_t>& res)
		{
			_dielHashes = res;
		}

		void ddPar::setDiels(const std::vector<std::string>& src)
		{
			_diels.clear();
			// Set individual dielectrics
			for (auto &file : src)
			{
				boost::shared_ptr<ddParParsers::ddParLineSimple<std::string> > res
					(new ddParParsers::ddParLineSimple<std::string>(ddParParsers::IREFR));
				res->set(file);
				_diels.push_back(res);
			}

			// And update the count
			boost::shared_ptr< ddParParsers::ddParLineSimple<int> > line
				(new ddParParsers::ddParLineSimple<int>(ddParParsers::NCOMP));
			line->set((int)_diels.size());
			insertKey(ddParParsers::NCOMP, boost::static_pointer_cast< ddParParsers::ddParLine >(line));
		}

		void ddPar::populateDefaults(bool overwrite, const std::string &src) const
		{
			// Populates missing items for this version with default
			// entries. Used when converting between ddscat file versions.
			// Also used when file information is incomplete.
			using namespace rtmath::ddscat::ddParParsers;
			using namespace boost::filesystem;
			using namespace std;
			// Let's take the default entries from a default scattering file.
			// I don't want to hardcode these values, plus, by varying the path, 
			// this improves scriptability...

			boost::shared_ptr<ddPar> basep; // basep is a copy with auto-deletion
			if (src == "")
			{
				basep = boost::shared_ptr<ddPar>(new ddPar(*(defaultInstance())));
			}
			else
			{
				if (boost::filesystem::exists(path(src)))
				{
					basep = boost::shared_ptr<ddPar>(new ddPar(src));
				}
				else {
					throw rtmath::debug::xMissingFile(src.c_str());
				}
			}
			ddPar &base = *basep;

			for (auto it = base._parsedData.begin(); it != base._parsedData.end(); it++)
			{
				// If overwrite, then overwrite any existing key
				// If not, and key exists, skip to next one
				// If key does not exist, add it
				if (this->_parsedData.count(it->first))
				{
					if (overwrite)
					{
						this->_parsedData.erase(it->first);
					}
					else {
						continue;
					}
				}
				this->_parsedData[it->first] = it->second;
			}
			for (auto it = base._scaPlanes.begin(); it != base._scaPlanes.end(); it++)
			{
				// If overwrite, then overwrite any existing key
				// If not, and key exists, skip to next one
				// If key does not exist, add it
				if (this->_scaPlanes.count(it->first))
				{
					if (overwrite)
					{
						this->_scaPlanes.erase(it->first);
					}
					else {
						continue;
					}
				}
				this->_scaPlanes[it->first] = it->second;
			}
		}



	} // end namespace ddscat
} // end rtmath


