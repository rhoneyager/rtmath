

#include <boost/math/constants/constants.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/complex.hpp>

#include <Ryan_Serialization/serialization.h>
#include <tmatrix/tmatrix.h>

#include "../../rtmath/rtmath/Serialization/serialization_macros.h"
#include "dataset.h"
#include "run.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"

#include "../../rtmath/rtmath/defs.h"



using boost::shared_ptr;
using namespace std;

ddPoint::ddPointSet ddPoint::genFromDataset(const dataset &d)
{
	ddPointSet res;
	res.reserve(d.ddloaded.size());

	for (const auto &spt : d.ddloaded)
	{
		boost::shared_ptr<ddPoint> pt(new ddPoint);
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
		pt->volFrac = d.stats->Sellipsoid_rms.f;

		/// \todo Allow for other determinations of aspect ratio
		/// \todo Check this
		pt->ddAsp = d.stats->calcStatsRot(0,0,0)->as_rms(0,2);

		res.push_back(boost::shared_ptr<const ddPoint>(pt));
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

tmRun::tmRun(const ddPoint::ddPointSet &src)
	: sources(src), scaleAeff(1), scaleF(1),
	errQabs(0), errQbk(0), errQsca(0), errQext(0)
{}

boost::shared_ptr<tmRun> tmRun::run()
{
	errQabs = 0;
	errQbk = 0;
	errQsca = 0;
	errQext = 0;

	// Iterate over each ddPoint, given the specified scaling factors, and perform a tmm run
	derived.reserve(sources.size());
	for (const auto &spt : sources)
	{
		boost::shared_ptr<tmPoint> pt(new tmPoint);
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
		tb.EPS = spt->ddAsp;

		boost::shared_ptr<const OriTmatrix> to(OriTmatrix::calc(tb));
		pt->tmQabs = to->qext - to->qsca;
		pt->tmQext = to->qext;
		pt->tmQsca = to->qsca;
		pt->tmQbk = tmatrix::getDifferentialBackscatterCrossSectionUnpol(to);

		// Figure out the error terma
		pt->errQabs = pow(pt->tmQabs - spt->ddQabs, 2.0f);
		pt->errQbk = pow(pt->tmQbk - spt->ddQbk, 2.0f);
		pt->errQext = pow(pt->tmQext - spt->ddQext, 2.0f);
		pt->errQsca = pow(pt->tmQsca - spt->ddQsca, 2.0f);

		errQabs += pt->errQabs;
		errQbk += pt->errQbk;
		errQext += pt->errQext;
		errQsca += pt->errQsca;

		// Store the tmPoint
		/// \todo Store the raw tmm results?
		derived.push_back(pt);
	}

	return this->shared_from_this();
}

boost::shared_ptr<tmRun> tmRun::genTMrun(
	const ddPoint::ddPointSet& p,
	float scaleAeff,
	float scaleF)
{
	boost::shared_ptr<tmRun> t(new tmRun(p));
	t->scaleAeff = scaleAeff;
	t->scaleF = scaleF;
	return t;
}


template<class Archive>
void ddPoint::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("ddQbk", ddQbk);
	ar & boost::serialization::make_nvp("ddQsca", ddQsca);
	ar & boost::serialization::make_nvp("ddQabs", ddQabs);
	ar & boost::serialization::make_nvp("ddQext", ddQext);
	ar & boost::serialization::make_nvp("ddAeff", ddAeff);
	ar & boost::serialization::make_nvp("ddWave", ddWave);
	ar & boost::serialization::make_nvp("ddSizeP", ddSizeP);
	ar & boost::serialization::make_nvp("ddM", ddM);
	ar & boost::serialization::make_nvp("volFrac", volFrac);
	ar & boost::serialization::make_nvp("ddAsp", ddAsp);
}

template<class Archive>
void tmPoint::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("source", source);
	ar & boost::serialization::make_nvp("tmQbk", tmQbk);
	ar & boost::serialization::make_nvp("tmQsca", tmQsca);
	ar & boost::serialization::make_nvp("tmQabs", tmQabs);
	ar & boost::serialization::make_nvp("tmQext", tmQext);
	ar & boost::serialization::make_nvp("scaleAeff", scaleAeff);
	ar & boost::serialization::make_nvp("scaleF", scaleF);
	ar & boost::serialization::make_nvp("tmAeff", tmAeff);
	ar & boost::serialization::make_nvp("tmF", tmF);
	ar & boost::serialization::make_nvp("tmM", tmM);
	ar & boost::serialization::make_nvp("errQabs", errQabs);
	ar & boost::serialization::make_nvp("errQsca", errQsca);
	ar & boost::serialization::make_nvp("errQbk", errQbk);
	ar & boost::serialization::make_nvp("errQext", errQext);
}

template<class Archive>
void tmRun::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("sources", sources);
	ar & boost::serialization::make_nvp("derived", derived);
	ar & boost::serialization::make_nvp("scaleAeff", scaleAeff);
	ar & boost::serialization::make_nvp("scaleF", scaleF);
	ar & boost::serialization::make_nvp("errQabs", errQabs);
	ar & boost::serialization::make_nvp("errQbk", errQbk);
	ar & boost::serialization::make_nvp("errQsca", errQsca);
	ar & boost::serialization::make_nvp("errQext", errQext);
}

template<class Archive>
void tmRunSet::serialize(Archive & ar, const unsigned int version)
{
	ar & boost::serialization::make_nvp("tmRuns", tmRuns);
	ar & boost::serialization::make_nvp("ddPoints", ddPoints);
	ar & boost::serialization::make_nvp("minQbk", minQbk);
	ar & boost::serialization::make_nvp("minQsca", minQsca);
	ar & boost::serialization::make_nvp("minQbkQsca", minQbkQsca);
}


BOOST_CLASS_EXPORT_IMPLEMENT(ddPoint);
BOOST_CLASS_EXPORT_IMPLEMENT(tmPoint);
BOOST_CLASS_EXPORT_IMPLEMENT(tmRun);
BOOST_CLASS_EXPORT_IMPLEMENT(tmRunSet);

EXPORTINTERNAL(ddPoint::serialize);
EXPORTINTERNAL(tmPoint::serialize);
EXPORTINTERNAL(tmRun::serialize);
EXPORTINTERNAL(tmRunSet::serialize);
