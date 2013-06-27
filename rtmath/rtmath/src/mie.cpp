#define EXPORTING_RTMATH
#include "../rtmath/mie/mie.h"
#include "../rtmath/mie/mie-Scalc.h"
#include "../rtmath/mie/mie-Qcalc.h"
#include <boost/math/constants/constants.hpp>

namespace rtmath
{
	namespace mie
	{
		mieBase::mieBase()
			: AXI(10), LAM(6.283), MRR(1.33),
			MRI(0),DDELT(0.001) {}
		bool mieBase::operator<(const mieBase &r) const
		{
#define LS(x) if(x != r.x) return x < r.x;
			LS(AXI);
			LS(LAM);
			LS(MRR);
			LS(MRI);
			LS(DDELT);
#undef LS
			return false;
		}

		mieParams::mieParams() : axi(0), lam(0), ddelt(0) {}

		boost::shared_ptr<const mieParams> mieParams::create(
			const mieBase &mb)
		{
			boost::shared_ptr<mieParams> res(new mieParams);
			res->axi = mb.AXI;
			res->lam = mb.LAM;
			res->ddelt = mb.DDELT;
			res->m = std::complex<double>(mb.MRR,abs(mb.MRI));
			return res;
		}

		boost::shared_ptr<const mieParams> mieParams::create(
			double axi, // equiv sphere radius
			double lam, // wavelength of incident light
			double mrr, // real refractive index
			double mri, // imaginary refractive index (positive)
			double ddelt)
		{
			boost::shared_ptr<mieParams> res(new mieParams);
			res->axi = axi;
			res->lam = lam;
			res->ddelt = ddelt;
			res->m = std::complex<double>(mrr,abs(mri));
			return res;
		}

		mieCalc::mieCalc() : qsca(0), qext(0), qbk(0), walb(0), qabs(0), g(0), sizep(0) {}

		boost::shared_ptr<const mieCalc> mieCalc::calc(const mieBase &r)
		{
			boost::shared_ptr<const mieParams> mp = mieParams::create(r);
			return calc(mp);
		}

		boost::shared_ptr<const mieCalc> mieCalc::calc(
			boost::shared_ptr<const mieParams> in)
		{
			boost::shared_ptr<mieCalc> res(new mieCalc);
			res->base = in;
			res->sizep = 2. * boost::math::constants::pi<double>() * in->axi / in->lam;
			mie::Qcalc qm(in->m);
			qm.calc(res->sizep,res->qext,res->qsca,res->qabs,res->qbk,res->g);
			res->walb = abs(res->qsca / res->qext);
			return res;
		}

		mieAngleRes::mieAngleRes() : theta(0), mu(0) {}

		bool mieAngleRes::operator<(const mieAngleRes &r) const
		{
#define CHECK(x) if (x < r.x) return x < r.x
			CHECK(theta);
#undef CHECK
			return false;
		}

		std::complex<double> mieAngleRes::getS(size_t i, size_t j) const
		{
			return S(i,j);
		}

		double mieAngleRes::getSnn(size_t i, size_t j) const
		{
			return P(i,j);
		}

		double mieAngleRes::getP(size_t i, size_t j) const
		{
			return P(i,j) * 4. / (mc->qext * mc->sizep * mc->sizep);
		}

		boost::shared_ptr<mieAngleRes> mieAngleRes::calc(
			boost::shared_ptr<const mieCalc> ot,
			double theta)
		{
			boost::shared_ptr<mieAngleRes> res(new mieAngleRes);
			res->mc = ot;
			res->theta = theta;

			// Perform calculation
			double pi = boost::math::constants::pi<double>();
			res->mu = cos(theta * pi / 180.);
			mie::Scalc sc(ot->base->m, ot->sizep);
			//Eigen::Matrix4d lSnn;
			//Eigen::Matrix2cd lSn;
			sc.calc(res->mu, res->S, res->P);

			return res;
		}


		double getDifferentialBackscatterCrossSectionUnpol(
			boost::shared_ptr<const mieCalc> tm)
		{
			// Needs wavenumber k, and S11 (from Mueller matrix)
			boost::shared_ptr<const mieAngleRes> sp = mieAngleRes::calc(tm, 180.0);
			double S11 = sp->getSnn(0,0);

			// k is provided indirectly by the t-matrix
			double wvlen = tm->base->lam; // units are unspecified
			// spectroscopic wavenumber = 1/wvlen, and angular wavenumber is 2pi/wvlen
			// we want k, which is the angular wavenumber
			const double pi = boost::math::constants::pi<double>();
			double k = 2. * pi / wvlen;

			double dBkSc = S11 / (k*k);
			return dBkSc;
		}
	}
}
