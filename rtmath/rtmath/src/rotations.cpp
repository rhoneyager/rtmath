#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>

#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/error/error.h"
#include "../../rtmath/rtmath/command.h"

namespace rtmath {
	namespace ddscat {

		rotationsBase::~rotationsBase()
		{
		}

		rotations::rotations()
		{
		}

		rotations::rotations(double bMin, double bMax, size_t bN,
				double tMin, double tMax, size_t tN,
				double pMin, double pMax, size_t pN)
		{
			_bMin = bMin;
			_bMax = bMax;
			_bN = bN;
			_tMin = tMin;
			_tMax = tMax;
			_tN = tN;
			_pMin = pMin;
			_pMax = pMax;
			_pN = pN;
		}

		boost::shared_ptr<rotations> rotations::create()
		{
			boost::shared_ptr<rotations> res(new rotations());
			return res;
		}

		boost::shared_ptr<rotations> rotations::create(
			double bMin, double bMax, size_t bN,
			double tMin, double tMax, size_t tN,
			double pMin, double pMax, size_t pN)
		{
			boost::shared_ptr<rotations> res(new rotations(
				bMin, bMax, bN,
				tMin, tMax, tN,
				pMin, pMax, pN));
			return res;
		}

		boost::shared_ptr<rotations> rotations::create(const ddPar &src)
		{
			boost::shared_ptr<rotations> res(new rotations(src));
			return res;
		}

		rotations::~rotations()
		{
		}

		bool rotations::operator==(const rotations &rhs) const
		{
			if (_bMin != rhs._bMin) return false;
			if (_bMax != rhs._bMax) return false;
			if (_bN != rhs._bN) return false;
			if (_tMin != rhs._tMin) return false;
			if (_tMax != rhs._tMax) return false;
			if (_tN != rhs._tN) return false;
			if (_pMin != rhs._pMin) return false;
			if (_pMax != rhs._pMax) return false;
			if (_pN != rhs._pN) return false;

			return true;
		}

		bool rotations::operator!=(const rotations &rhs) const
		{
			return !(operator==(rhs));
		}

		bool rotations::operator<(const rotations &rhs) const
		{
			if (_bMin != rhs._bMin) return _bMin < rhs._bMin;
			if (_bMax != rhs._bMax) return _bMax < rhs._bMax;
			if (_bN != rhs._bN) return _bN < rhs._bN;
			if (_tMin != rhs._tMin) return _tMin < rhs._tMin;
			if (_tMax != rhs._tMax) return _tMax < rhs._tMax;
			if (_tN != rhs._tN) return _tN < rhs._tN;
			if (_pMin != rhs._pMin) return _pMin < rhs._pMin;
			if (_pMax != rhs._pMax) return _pMax < rhs._pMax;
			if (_pN != rhs._pN) return _pN < rhs._pN;

			return false;
		}

		rotations::rotations(const ddPar &src)
		{
			using namespace boost;
			shared_ptr<const ddParParsers::ddParLine > ob, ot, op;
			rotations defaults;
			if (src.exists(ddParParsers::NBETA))
			{
				src.getKey(ddParParsers::NBETA, ob);
				shared_ptr<const ddParParsers::ddParLineMixed<double, size_t> > b = 
					static_pointer_cast<const ddParParsers::ddParLineMixed<double, size_t> >
					(ob);
				b->get<double>(0,_bMin);
				b->get<double>(1,_bMax);
				b->get<size_t>(2,_bN);
			} else {
				_bMin = defaults.bMin();
				_bMax = defaults.bMax();
				_bN = defaults.bN();
			}

			if (src.exists(ddParParsers::NTHETA))
			{
				src.getKey(ddParParsers::NTHETA, ot);
				shared_ptr<const ddParParsers::ddParLineMixed<double, size_t> > t = 
					static_pointer_cast<const ddParParsers::ddParLineMixed<double, size_t> >
					(ot);
				t->get<double>(0,_tMin);
				t->get<double>(1,_tMax);
				t->get<size_t>(2,_tN);
			} else {
				_tMin = defaults.tMin();
				_tMax = defaults.tMax();
				_tN = defaults.tN();
			}

			if (src.exists(ddParParsers::NPHI))
			{
				src.getKey(ddParParsers::NPHI, op);
				shared_ptr<const ddParParsers::ddParLineMixed<double, size_t> > p = 
					static_pointer_cast<const ddParParsers::ddParLineMixed<double, size_t> >
					(op);
				p->get<double>(0,_pMin);
				p->get<double>(1,_pMax);
				p->get<size_t>(2,_pN);
			} else {
				_pMin = defaults.pMin();
				_pMax = defaults.pMax();
				_pN = defaults.pN();
			}
		}

		void rotations::out(ddPar &dest) const
		{
			using namespace boost;
			shared_ptr<ddParParsers::ddParLineMixed<double, size_t> > 
				b (new ddParParsers::ddParLineMixed<double, size_t>(2,3, ddParParsers::NBETA) ), 
				t (new ddParParsers::ddParLineMixed<double, size_t>(2,3, ddParParsers::NTHETA) ), 
				p (new ddParParsers::ddParLineMixed<double, size_t>(2,3, ddParParsers::NPHI) );
			b->set<double>(0, _bMin);
			b->set<double>(1, _bMax);
			b->set<size_t>(2, _bN);
			t->set<double>(0, _tMin);
			t->set<double>(1, _tMax);
			t->set<size_t>(2, _tN);
			p->set<double>(0, _pMin);
			p->set<double>(1, _pMax);
			p->set<size_t>(2, _pN);

			dest.insertKey(ddParParsers::NBETA,static_pointer_cast<ddParParsers::ddParLine>(b));
			dest.insertKey(ddParParsers::NTHETA,static_pointer_cast<ddParParsers::ddParLine>(t));
			dest.insertKey(ddParParsers::NPHI,static_pointer_cast<ddParParsers::ddParLine>(p));
		}

		void rotations::betas(std::string &dest) const
		{
			using namespace std;
			ostringstream out;
			out << bMin() << ":" << bN() << ":" << bMax() << ":LIN";
			dest = out.str();
		}

		void rotations::thetas(std::string &dest) const
		{
			using namespace std;
			ostringstream out;
			out << tMin() << ":" << tN() << ":" << tMax() << ":COS";
			dest = out.str();
		}

		void rotations::phis(std::string &dest) const
		{
			using namespace std;
			ostringstream out;
			out << pMin() << ":" << pN() << ":" << pMax() << ":LIN";
			dest = out.str();
		}

		void rotations::betas(std::set<double> &b) const
		{
			using namespace std;
			string sbetas;
			betas(sbetas);
			rtmath::config::splitSet<double>(sbetas,b);
		}

		void rotations::thetas(std::set<double> &t) const
		{
			using namespace std;
			string sthetas;
			thetas(sthetas);
			rtmath::config::splitSet<double>(sthetas,t);
		}

		void rotations::phis(std::set<double> &p) const
		{
			using namespace std;
			string sphis;
			phis(sphis);
			rtmath::config::splitSet<double>(sphis,p);
		}
	}
}

