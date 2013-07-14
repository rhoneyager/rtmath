#include "dataset.h"
#include "run.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include <boost/math/constants/constants.hpp>
#include <tmatrix/tmatrix.h>

using boost::shared_ptr;
using namespace std;

vector<shared_ptr<const ddPoint> > ddPoint::genFromDataset(const dataset &d)
{
	vector<shared_ptr<const ddPoint> > res;
	res.reserve(d.ddloaded.size());

	for (const auto &spt : d.ddloaded)
	{
		shared_ptr<ddPoint> pt(new ddPoint);
		pt->ddAeff = spt->aeff();
		pt->ddWave = spt->wave();
		pt->ddQabs = spt->getStatEntry(rtmath::ddscat::QABSM);
		pt->ddQbk = spt->getStatEntry(rtmath::ddscat::QBKM);
		pt->ddQsca = spt->getStatEntry(rtmath::ddscat::QSCAM);
		pt->ddQext = spt->getStatEntry(rtmath::ddscat::QEXTM);

		const float pi = boost::math::constants::pi<float>();
		pt->ddSizeP = 2.f * pi * pt->ddAeff / pt->ddWave;
		
		// Thankfully, each .avg file provides m
		// It's a very odd key...
		pt->ddM = spt->getM();
		
		/// \todo Allow for other base volume fractions
		pt->volFrac = d.stats->f_ellipsoid_rms;

		res.push_back(shared_ptr<const ddPoint>(pt));
	}

	return res;
}

tmPoint::tmPoint() :
	tmQbk(0), tmQabs(0), tmQext(0), tmQsca(0),
	scaleAeff(1), scaleF(1),
	tmAeff(0), tmF(0),
	errQabs(0), errQsca(0), errQbk(0), errQext(0)
{
}

tmRun::tmRun(const std::vector<boost::shared_ptr<const ddPoint> >& src)
	: sources(src), scaleAeff(1), scaleF(1),
	errQabs(0), errQbk(0), errQsca(0), errQext(0)
{}

void tmRun::run()
{
	errQabs = 0;
	errQbk = 0;
	errQsca = 0;
	errQext = 0;

	// Iterate over each ddPoint, given the specified scaling factors, and perform a tmm run
	derived.reserve(sources.size());
	for (const auto &spt : sources)
	{
		shared_ptr<tmPoint> pt(new tmPoint);
		pt->source = spt;
		pt->scaleAeff = scaleAeff;
		pt->scaleF = scaleF;

		pt->tmAeff = spt->ddAeff * scaleAeff;
		pt->tmF = spt->volFrac * scaleF;

		/// \todo Scale based on scaling relation?
		pt->tmM = spt->ddM * complex<double>(pt->tmF,0);

		// Construct and execute the tmatrix run
		using namespace tmatrix;
		tmatrixBase tb;
		tb.AXI = pt->tmAeff;
		tb.LAM = spt->ddWave;
		tb.MRR = pt->tmM.real();
		tb.MRI = pt->tmM.imag();
		/// \todo Add eps!
		tb.EPS;

		shared_ptr<const OriTmatrix> to(OriTmatrix::calc(tb));
		pt->tmQabs = to->qext - to->qsca;
		pt->tmQext = to->qext;
		pt->tmQsca = to->qsca;
		pt->tmQbk = getDifferentialBackscatterCrossSectionUnpol(to);
	}
}
