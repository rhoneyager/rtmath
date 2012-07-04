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

		rotations::~rotations()
		{
		}

		rotations::rotations(const ddPar &src)
		{
			using namespace boost;
			shared_ptr<const ddParParsers::ddParLine > ob, ot, op;
			src.getKey(ddParParsers::NBETA, ob);
			shared_ptr<const ddParParsers::ddParLineMixed<double, size_t> > b = 
				static_pointer_cast<const ddParParsers::ddParLineMixed<double, size_t> >
				(ob);
			b->get<double>(0,_bMin);
			b->get<double>(1,_bMax);
			b->get<size_t>(2,_bN);

			src.getKey(ddParParsers::NTHETA, ot);
			shared_ptr<const ddParParsers::ddParLineMixed<double, size_t> > t = 
				static_pointer_cast<const ddParParsers::ddParLineMixed<double, size_t> >
				(ob);
			t->get<double>(0,_tMin);
			t->get<double>(1,_tMax);
			t->get<size_t>(2,_tN);

			src.getKey(ddParParsers::NPHI, op);
			shared_ptr<const ddParParsers::ddParLineMixed<double, size_t> > p = 
				static_pointer_cast<const ddParParsers::ddParLineMixed<double, size_t> >
				(ob);
			p->get<double>(0,_pMin);
			p->get<double>(1,_pMax);
			p->get<size_t>(2,_pN);
		}

		void rotations::out(ddPar &dest) const
		{
			using namespace boost;
			shared_ptr<ddParParsers::ddParLineMixed<double, size_t> > b, t, p;
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
	}
}

