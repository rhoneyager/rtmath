
#pragma warning( disable : 4996 ) // SCL_SECURE_NO_WARNINGS
#define EXPORTING_TMATRIX
#include <algorithm>
#include <complex>
#include <iostream>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <boost/cstdint.hpp>
#include <boost/checked_delete.hpp>
#include <boost/math/constants/constants.hpp>
#include <Ryan_Debug/error.h>
#include "../headers/par.h"
#include "../headers/fortrancommon.h"
#include "../headers/tmatrix.h"
#include "cmake-settings.h"

//#pragma comment(lib, "test")

extern "C"
{
	extern void tmatrix_double_();
	extern void tmatrix_amplpha_();
	extern void check_aligns_(int *aligned);

	void throw_int(int *a)
	{
		//throw *a;
		//throw std::domain_error(string_);
	}
};

namespace tmatrix
{
	void throw_tmerror(int errcode) {
		if (errcode==1) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("CONVERGENCE IS NOT OBTAINED FOR NPN1")
			<< ::Ryan_Debug::error::symbol_name("NPN1")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);

		if (errcode==2) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("NGAUSS IS GREATER THAN NPNG1")
			<< ::Ryan_Debug::error::symbol_name("NGAUSS")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);

		if (errcode==3) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("CONVERGENCE IS NOT OBTAINED FOR NPN1/NMA")
			<< ::Ryan_Debug::error::symbol_name("NPN1/NMA")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);


		RDthrow(::Ryan_Debug::error::xOtherError())
			<< ::Ryan_Debug::error::otherErrorText("Unknown/unhandled error in the Mishchenko T-matrix code")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);
	}

	tmatrixBase::tmatrixBase()
		: AXI(10),
		RAT(0.1),
		LAM(6.283185307),
		MRR(1.5),
		MRI(0.02),
		EPS(0.5),
		DDELT(0.001),
		ALPHA(145),
		BETA(52),
		NP(-1),
		NDGS(2)
	{}

	bool tmatrixBase::operator<(const tmatrixBase &r) const
	{
#define CHECK(x) if (x < r.x) return x < r.x
		CHECK(AXI);
		CHECK(RAT);
		CHECK(LAM);
		CHECK(MRR);
		CHECK(MRI);
		CHECK(EPS);
		CHECK(DDELT);
		CHECK(ALPHA);
		CHECK(BETA);
		CHECK(NP);
		CHECK(NDGS);
#undef CHECK
		return false;
	}

	tmatrixParams::tmatrixParams()
		: axi(0), rat(0), lam(0), eps(0), 
		np(0), ddelt(0), ndgs(0)
	{
	}

	void tmatrixParams::apply(double alpha, double beta) const
	{
		inputs_.AXI = axi;
		inputs_.RAT = rat;
		inputs_.LAM = lam;
		inputs_.MRR = m.real();
		inputs_.MRI = m.imag();
		inputs_.EPS = eps;
		inputsb_.NP = np;
		inputs_.DDELT = ddelt;
		inputsb_.NDGS = ndgs;
		inputs_.ALPHA = alpha;
		inputs_.BETA = beta;
	}

	bool tmatrixParams::needApply(double alpha, double beta) const
	{
		if (inputs_.AXI != axi) return true;
		if (inputs_.RAT != rat) return true;
		if (inputs_.LAM != lam) return true;
		if (inputs_.MRR != m.real()) return true;
		if (inputs_.MRI != m.imag()) return true;
		if (inputs_.EPS != eps) return true;
		if (inputsb_.NP != np) return true;
		if (inputs_.DDELT != ddelt) return true;
		if (inputsb_.NDGS != ndgs) return true;
		if (inputs_.ALPHA != alpha) return true;
		if (inputs_.BETA != beta) return true;

		return false;
	}

	bool tmatrixParams::operator==(const tmatrixParams &rhs) const
	{
		if (rhs.axi != axi) return false;
		if (rhs.rat != rat) return false;
		if (rhs.lam != lam) return false;
		if (rhs.m != m) return false;
		if (rhs.eps != eps) return false;
		if (rhs.np != np) return false;
		if (rhs.ddelt != ddelt) return false;
		if (rhs.ndgs != ndgs) return false;

		return true;
	}

	bool tmatrixParams::operator<(const tmatrixParams &rhs) const
	{
		if (rhs.axi != axi) return (rhs.axi > axi);
		if (rhs.rat != rat) return (rhs.rat > rat);
		if (rhs.lam != lam) return (rhs.lam > lam);
		if (rhs.m.real() != m.real()) return (rhs.m.real() > m.real());
		if (rhs.m.imag() != m.imag()) return (rhs.m.real() > m.real());
		if (rhs.eps != eps) return (rhs.eps > eps);
		if (rhs.np != np) return (rhs.np > np);
		if (rhs.ddelt != ddelt) return (rhs.ddelt > ddelt);
		if (rhs.ndgs != ndgs) return (rhs.ndgs > ndgs);

		return false;
	}

	boost::shared_ptr<const tmatrixParams> tmatrixParams::create(const tmatrixBase& b)
	{
		boost::shared_ptr<tmatrixParams> t(new tmatrixParams);
		t->axi = b.AXI;
		t->rat = b.RAT;
		t->lam = b.LAM;
		t->m = std::complex<double>(b.MRR,b.MRI);
		t->eps = b.EPS;
		t->np = b.NP;
		t->ddelt = b.DDELT;
		t->ndgs = b.NDGS;
		return t;
	}

	boost::shared_ptr<const tmatrixParams> tmatrixParams::create(
		double AXI,
		double RAT,
		double LAM,
		double MRR,
		double MRI,
		double EPS,
		boost::int32_t NP,
		double DDELT,
		boost::int32_t NDGS
		)
	{
		boost::shared_ptr<tmatrixParams> t(new tmatrixParams);
		t->axi = AXI;
		t->rat = RAT;
		t->lam = LAM;
		t->m = std::complex<double>(MRR,MRI);
		t->eps = EPS;
		t->np = NP;
		t->ddelt = DDELT;
		t->ndgs = NDGS;
		return t;
	}

	OriTmatrix::OriTmatrix()
		: qsca(0), qext(0), walb(0), time(0),
		alpha(0), beta(0), projArea(0), nmax(0)
	{
		base = boost::shared_ptr<const tmatrixParams>(new tmatrixParams);
	}

	boost::shared_ptr<const OriTmatrix> OriTmatrix::calc(
		boost::shared_ptr<const tmatrixParams> in,
		double alpha, double beta)
	{
		boost::shared_ptr<OriTmatrix> res(new OriTmatrix);
		res->base = in;
		res->alpha = alpha;
		res->beta = beta;
		
		if (in->needApply(alpha,beta))
		{
			in->apply(alpha,beta);
			tmatrix_double_();
		}
		res->import();
		res->calcParams();
		return res;
	}

	boost::shared_ptr<const OriTmatrix> OriTmatrix::calc(
		const tmatrixBase& b)
	{
		boost::shared_ptr<OriTmatrix> res(new OriTmatrix);
		res->base = tmatrixParams::create(b);
		res->alpha = b.ALPHA;
		res->beta = b.BETA;
		
		if (res->base->needApply(res->alpha,res->beta))
		{
			res->base->apply(res->alpha,res->beta);
			tmatrix_double_();
		}
		res->import();
		res->calcParams();
		return res;
	}

	void OriTmatrix::import()
	{
		using namespace std;
		double pi = boost::math::constants::pi<double>();
		double sizep = 2. * pi * this->base->axi / this->base->lam;
		double scale = 2. / (sizep * sizep);
		qsca = outputs_.QSCA * scale;
		qext = abs(outputs_.QEXT * scale);
		walb = abs(outputs_.WALB);
		time = outputs_.TIME;
		nmax = outputs_.NMAX;

		int errcode = 0;
		errcode = outputs_.QERROR;
		if (errcode)
			throw_tmerror(errcode);
	}

	void OriTmatrix::calcParams()
	{
		projArea = calcProjArea(this->base, this->beta);
	}

	double calcProjArea(boost::shared_ptr<const tmatrixParams> p, double beta)
	{
		using namespace std;
		const double pi = boost::math::constants::pi<double>();
		const double &eps = p->eps;
		const double &aeff = p->axi;

		// a=b, and c is independent. eps=a/c is known.
		const double a = pow(eps,1./3.) * aeff;
		const double c = pow(eps,-2./3.) * aeff;
		const double Vell = (4./3.)*pi*pow(aeff,3.);

		const double gamma=pi*beta/180.; // To keep from changing the formula, gamma is in radians
		const double h=2.*a/sqrt(1+ ((eps*eps)-1.)*pow(cos(gamma),2.));

		// h=3V/2E, E is the projected area
		double projArea = 3.*Vell/(2.*h);
		return projArea;
	}

	OriAngleRes::OriAngleRes()
		: theta(0), theta0(0), phi(0), phi0(0) 
	{
		S = boost::shared_ptr<const std::vector<std::complex<double> > >(
			new std::vector<std::complex<double> >(4));

		P = boost::shared_ptr<const std::vector<double> >(
			new std::vector<double>(16));

		tm = boost::shared_ptr<const OriTmatrix>(new OriTmatrix);
	}

	bool OriAngleRes::operator<(const OriAngleRes &r) const
	{
#define CHECK(x) if (x < r.x) return x < r.x
		CHECK(theta);
		CHECK(theta0);
		CHECK(phi);
		CHECK(phi0);
#undef CHECK
		return false;
	}

	std::complex<double> OriAngleRes::getS(size_t i, size_t j) const
	{
		size_t index = (2*i) + j;
		std::complex<double> r = S->at(index);
		return std::complex<double>(static_cast<double>(r.real()), 
			static_cast<double>(r.imag()));
	}

	double OriAngleRes::getP(size_t i, size_t j) const
	{
		size_t index = (4*i) + j;
		return static_cast<double>(P->at(index));
	}

	boost::shared_ptr<OriAngleRes> OriAngleRes::calc(
		boost::shared_ptr<const OriTmatrix> ot,
		double theta, double theta0,
		double phi, double phi0)
	{
		boost::shared_ptr<OriAngleRes> res(new OriAngleRes);
		res->tm = ot;
		res->theta = theta;
		res->theta0 = theta0;
		res->phi = phi;
		res->phi0 = phi0;
		
		// Save to common block
		inputsang_.THET0 = theta0;
		inputsang_.THET = theta;
		inputsang_.PHI0 = phi0;
		inputsang_.PHI = phi;

		// If the necessary tmatrix has already been computed,
		// preserve it and do not call tmatrix_double_
		if (res->tm->base->needApply(res->tm->alpha,res->tm->beta))
		{
			res->tm->base->apply(res->tm->alpha, res->tm->beta);

			// Execute tmatrix code
			tmatrix_double_(); // to get the tmatrix
		}
		tmatrix_amplpha_(); // to compute the angles

		// Import function can just be included here for now
		using namespace std;
		boost::shared_ptr<vector<complex<double> > > iS (
			boost::const_pointer_cast<vector<complex<double> > >(res->S) );

		boost::shared_ptr<vector<double> > iP (
			boost::const_pointer_cast<vector<double> >(res->P) );

		// Technically, double and double are the same type, so I could get away without 
		// doing this.
		/*
		auto tDc = [](const complex<double> &i) -> complex<double>
		{
			return complex<double>(double(i.real()), double(i.imag()) );
		}; */
		iS->operator[](0) = (complex<double>(outputsamp_.OS[0],outputsamp_.OS[1]));
		iS->operator[](1) = (complex<double>(outputsamp_.OS[2],outputsamp_.OS[3]));
		iS->operator[](2) = (complex<double>(outputsamp_.OS[4],outputsamp_.OS[5]));
		iS->operator[](3) = (complex<double>(outputsamp_.OS[6],outputsamp_.OS[7]));
	
		std::copy_n(outputsamp_.OP,16,iP->begin());

		int errcode = 0;
		errcode = outputsamp_.AERROR;
		if (errcode) throw_tmerror(errcode);

		return res;
	}

	double OriAngleRes::getFlippedP(size_t i, size_t j) const
	{
		using namespace std;
		boost::shared_ptr< vector<complex<double> > > fS(new vector<complex<double> >(4));
		for (size_t k=0; k<4; k++)
			fS->operator[](k) = complex<double>(S->at(k).imag(),S->at(k).real());

		boost::shared_ptr< const vector<double> > flipP;
		flipP = getP(fS);

		size_t index = (4*i) + j;
		return static_cast<double>(flipP->at(index));
	}

	double getDifferentialBackscatterCrossSectionUnpol(
		boost::shared_ptr<const OriTmatrix> tm, bool flip)
	{
		// Needs wavenumber k, and S11 (from Mueller matrix)
		auto sp = OriAngleRes::calc(tm, 180.0, 0, 0, 0);
		double S11 = (flip) ? sp->getFlippedP(0,0) : sp->getP(0,0);

		// k is provided indirectly by the t-matrix
		//double wvlen = tm->base->lam; // units are unspecified
		// spectroscopic wavenumber = 1/wvlen, and angular wavenumber is 2pi/wvlen
		// we want k, which is the angular wavenumber
		//const double pi = boost::math::constants::pi<double>();
		//double k = 2. * pi / wvlen;

		double dBkSc = S11 * 4.0 / pow(tm->base->axi,2.0);

		// From here, rescale the backscatter equation
		//double scale = pow(2.*pi/wvlen,2.);
		//dBkSc *= scale; // Now agrees with ddscat initial units

		// TODO: physical units with same scale and dimensions as the other cross-sections

		return dBkSc;
	}

	boost::shared_ptr< const std::vector<double> > OriAngleRes::getP(
		const boost::shared_ptr< const std::vector<std::complex<double> > > &S)
	{
		/*
      Z11=0.5D0*(S11*DCONJG(S11)+S12*DCONJG(S12)
     &          +S21*DCONJG(S21)+S22*DCONJG(S22))
      Z12=0.5D0*(S11*DCONJG(S11)-S12*DCONJG(S12)
     &          +S21*DCONJG(S21)-S22*DCONJG(S22))
      Z13=-S11*DCONJG(S12)-S22*DCONJG(S21)
      Z14=(0D0,1D0)*(S11*DCONJG(S12)-S22*DCONJG(S21))
      Z21=0.5D0*(S11*DCONJG(S11)+S12*DCONJG(S12)
     &          -S21*DCONJG(S21)-S22*DCONJG(S22))
      Z22=0.5D0*(S11*DCONJG(S11)-S12*DCONJG(S12)
     &          -S21*DCONJG(S21)+S22*DCONJG(S22))
      Z23=-S11*DCONJG(S12)+S22*DCONJG(S21)
      Z24=(0D0,1D0)*(S11*DCONJG(S12)+S22*DCONJG(S21))
      Z31=-S11*DCONJG(S21)-S22*DCONJG(S12)
      Z32=-S11*DCONJG(S21)+S22*DCONJG(S12)
      Z33=S11*DCONJG(S22)+S12*DCONJG(S21)
      Z34=(0D0,-1D0)*(S11*DCONJG(S22)+S21*DCONJG(S12))

      Z41=(0D0,1D0)*(S21*DCONJG(S11)+S22*DCONJG(S12))
      Z42=(0D0,1D0)*(S21*DCONJG(S11)-S22*DCONJG(S12))
      Z43=(0D0,-1D0)*(S22*DCONJG(S11)-S12*DCONJG(S21))
      Z44=S22*DCONJG(S11)-S12*DCONJG(S21)
	  */
		using namespace std;
		std::complex<double> tS11 = S->at(0), tS12 = S->at(1),
			tS21 = S->at(2), tS22 = S->at(3), ti(double(0.),double(1.));
		auto conv = [](const std::complex<double> t)
		{
			return std::complex<double>((double)(t.real()),(double)(t.imag()));
		};
		std::complex<double> S11=conv(tS11), S12=conv(tS12), S21=conv(tS21),S22=conv(tS22),
			i=conv(ti);

		boost::shared_ptr< std::vector<double> > P(new vector<double>(16));
		P->operator[](0) = 0.5 * (norm(S11) + norm(S12) + norm(S21) + norm(S22) );
		P->operator[](1) = 0.5 * (norm(S11) - norm(S12) + norm(S21) - norm(S22) );
		P->operator[](2) = (-(S11*conj(S12)) - (S22*conj(S21))).real();
		P->operator[](3) = (i* ((S11*conj(S12))-(S22*conj(S21))) ).real();

		P->operator[](4) = 0.5 * (norm(S11) + norm(S12) - norm(S21) - norm(S22) );
		P->operator[](5) = 0.5 * (norm(S11) - norm(S12) - norm(S21) + norm(S22) );
		P->operator[](6) = (-(S11*conj(S12)) + (S22*conj(S21))).real();
		P->operator[](7) = (i* ((S11*conj(S12))+(S22*conj(S21))) ).real();

		P->operator[](8) = (-(S11*conj(S21)) - (S22*conj(S12))).real();
		P->operator[](9) = (-(S11*conj(S21)) + (S22*conj(S12))).real();
		P->operator[](10) = ((S11*conj(S22)) + (S12*conj(S21))).real();
		P->operator[](11) = (-i* ((S11*conj(S22))+(S21*conj(S12))) ).real();

		P->operator[](12) = (i* ((S21*conj(S11))+(S22*conj(S12))) ).real();
		P->operator[](13) = (i* ((S21*conj(S11))-(S22*conj(S12))) ).real();
		P->operator[](14) = (-i* ((S22*conj(S11))-(S12*conj(S21))) ).real();
		P->operator[](15) = ((S22*conj(S11)) - (S12*conj(S21))).real();

		boost::shared_ptr< const std::vector<double> > r(P);
		return r;
	}


	void printDebugInfo(std::ostream &out)
	{
		using std::cerr;
		using std::string;
		using std::endl;
		out << "Ryan_Tmatrix information\n"
			<< "Version: " << RYAN_TMATRIX_MAJOR << "." << RYAN_TMATRIX_MINOR << "."
			<< RYAN_TMATRIX_REVISION << "." << RYAN_TMATRIX_SVNREVISION << endl;
	}

}


