
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
#include "../../../../../ryan-debug/Ryan_Debug/error.h"
#include "../headers/par.h"
#include "../headers/fortrancommon.h"
#include "../headers/tmatrix.h"
//#include "cmake-settings.h"

//#pragma comment(lib, "test")

extern "C"
{
	extern void tmatrix_iso_double_();
	extern void check_aligns_(int *aligned);

	void throw_int(int *a)
	{
		//throw *a;
		//throw std::domain_error(string_);
	}
};

namespace tmatrix_random
{
	void throw_tmerror(int errcode) {
		if (errcode==1) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("CONVERGENCE IS NOT OBTAINED FOR NPN1")
			<< ::Ryan_Debug::error::symbol_name("NPN1")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);

		if (errcode==2 || errcode == 12) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("NGAUSS IS GREATER THAN NPNG1")
			<< ::Ryan_Debug::error::symbol_name("NGAUSS")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);

		if (errcode==3) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("CONVERGENCE IS NOT OBTAINED FOR NPN1/NMA")
			<< ::Ryan_Debug::error::symbol_name("NPN1/NMA")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);

		if (errcode==10) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("NK > 1000")
			<< ::Ryan_Debug::error::symbol_name("NK")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);

		if (errcode==43) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("NPN1==NMA")
			<< ::Ryan_Debug::error::symbol_name("NPN1,NMA")
			<< ::Ryan_Debug::error::otherErrorCode(errcode);

		if (errcode==13) RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("NMAX1>NPN4")
			<< ::Ryan_Debug::error::symbol_name("NMAX1,NPN4")
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
		GAM(0.5),
		DDELT(0.001),
		R1(0.1),
		R2(0.1),
		AXMAX(10),
		NP(-1),
		NDGS(2),
		NDISTR(4),
		NPNAX(1),
		NPNA(19),
		B(0.1),
		NKMAX(-1)
	{}

	bool tmatrixBase::operator<(const tmatrixBase &r) const
	{
#define CHECK(x) if (x != r.x) return x < r.x
		CHECK(AXI);
		CHECK(RAT);
		CHECK(LAM);
		CHECK(GAM);
		CHECK(MRR);
		CHECK(MRI);
		CHECK(EPS);
		CHECK(DDELT);
		CHECK(R1);
		CHECK(R2);
		CHECK(AXMAX);
		CHECK(B);
		CHECK(NP);
		CHECK(NDGS);
		CHECK(NDISTR);
		CHECK(NPNAX);
		CHECK(NPNA);
		CHECK(NKMAX);
#undef CHECK
		return false;
	}

	tmatrixParams::tmatrixParams()
		: axi(100), rat(1), lam(1635.44), eps(1.00001), gam(0.5),
		np(-1), ddelt(0.0001), ndgs(2), r1(99.9999), r2(100.0001), axmax(100), b(0.1),
		ndistr(4), npnax(1), npna(3), nkmax(-1), dbg(0)
	{
		m = std::complex<double>(1.783,0.003862);
	}

	void tmatrixParams::apply() const
	{
		//std::cerr << "AXMAX " << axmax << ", RAT " << rat << ", LAM " << lam
		//	<< ", MRR " << m.real() << ", MRI " << std::abs(m.imag()) << ", EPS " << eps
		//	<< ", DDELT " << ddelt << ", GAM " << gam << ", B " << b << std::endl;
		inputsiso_.AXMAX = axmax;
		inputsiso_.RAT = rat;
		inputsiso_.LAM = lam;
		inputsiso_.MRR = m.real();
		inputsiso_.MRI = std::abs(m.imag());
		inputsiso_.EPS = eps;
		inputsiso_.DDELT = ddelt;
		inputsiso_.GAM = gam;
		inputsiso_.B = b;
		inputsbiso_.NP = np;
		inputsbiso_.NDGS = ndgs;
		inputsbiso_.NDISTR = ndistr;
		inputsbiso_.NPNAX = npnax;
		inputsbiso_.NPNA = npna;
		inputsbiso_.NKMAX = nkmax;
		inputsbiso_.DBG = dbg;
	}

	bool tmatrixParams::needApply() const
	{
		if (inputsiso_.AXMAX != axi) return true;
		if (inputsiso_.RAT != rat) return true;
		if (inputsiso_.LAM != lam) return true;
		if (inputsiso_.MRR != m.real()) return true;
		if (inputsiso_.MRI != std::abs(m.imag())) return true;
		if (inputsiso_.EPS != eps) return true;
		if (inputsiso_.DDELT != ddelt) return true;
		if (inputsiso_.AXMAX != axmax) return true;
		if (inputsiso_.B != b) return true;
		if (inputsbiso_.NP != np) return true;
		if (inputsbiso_.NDGS != ndgs) return true;
		if (inputsbiso_.NDISTR != ndistr) return true;
		if (inputsbiso_.NPNAX != npnax) return true;
		if (inputsbiso_.NPNA != npna) return true;
		if (inputsbiso_.NKMAX != nkmax) return true;
		return false;
	}

	bool tmatrixParams::operator==(const tmatrixParams &rhs) const
	{
		if (rhs.axi != axi) return false;
		if (rhs.rat != rat) return false;
		if (rhs.lam != lam) return false;
		if (rhs.eps != eps) return false;
		if (rhs.ddelt != ddelt) return false;
		if (rhs.r1 != r1) return false;
		if (rhs.r2 != r2) return false;
		if (rhs.axmax != axmax) return false;
		if (rhs.b != b) return false;
		if (rhs.m != m) return false;
		if (rhs.np != np) return false;
		if (rhs.ndgs != ndgs) return false;
		if (rhs.ndistr != ndistr) return false;
		if (rhs.npnax != npnax) return false;
		if (rhs.npna != npna) return false;
		if (rhs.nkmax != nkmax) return false;

		return true;
	}

	bool tmatrixParams::operator<(const tmatrixParams &rhs) const
	{
#define CHECK2(x) if(rhs. x != x) return (rhs. x > x);
		CHECK2(axi);
		CHECK2(rat);
		CHECK2(lam);
		CHECK2(eps);
		CHECK2(ddelt);
		CHECK2(r1);
		CHECK2(r2);
		CHECK2(axmax);
		CHECK2(b);
		CHECK2(m.real());
		CHECK2(m.imag());
		CHECK2(np);
		CHECK2(ndgs);
		CHECK2(ndistr);
		CHECK2(npnax);
		CHECK2(npna);
		CHECK2(nkmax);

		return false;
	}

	boost::shared_ptr<const tmatrixParams> tmatrixParams::create(const tmatrixBase& b)
	{
		boost::shared_ptr<tmatrixParams> t(new tmatrixParams);
		t->axi = b.AXI;
		t->rat = b.RAT;
		t->lam = b.LAM;
		t->eps = b.EPS;
		t->ddelt = b.DDELT;
		t->r1 = b.R1;
		t->r2 = b.R2;
		t->axmax = b.AXMAX;
		t->b = b.B;
		t->m = std::complex<double>(b.MRR,std::abs(b.MRI));
		t->np = b.NP;
		t->ndgs = b.NDGS;
		t->ndistr = b.NDISTR;
		t->npnax = b.NPNAX;
		t->npna = b.NPNA;
		t->nkmax = b.NKMAX;
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
		t->axmax = AXI;
		t->rat = RAT;
		t->lam = LAM;
		t->m = std::complex<double>(MRR,MRI);
		t->eps = EPS;
		t->np = NP;
		t->ddelt = DDELT;
		t->ndgs = NDGS;
		// Other parameters are set by the constructor.
		return t;
	}

	struct isoRes {
		isores_internal data;
		isoRes(const isores_internal &in) {
			data.CSCAT = in.CSCAT;
			data.CEXTIN = in.CEXTIN;
			data.WALB = in.WALB;
			data.TIME = in.TIME;
			data.NMAX = in.NMAX;
			data.ASYMM = in.ASYMM;
			data.QERROR = in.QERROR;

			//std::copy_n(in.ALPH1, defs::tmd::NPL, data.ALPH1);
			//std::copy_n(in.ALPH2, defs::tmd::NPL, data.ALPH2);
			//std::copy_n(in.ALPH3, defs::tmd::NPL, data.ALPH3);
			//std::copy_n(in.ALPH4, defs::tmd::NPL, data.ALPH4);
			//std::copy_n(in.BET1, defs::tmd::NPL, data.BET1);
			//std::copy_n(in.BET2, defs::tmd::NPL, data.BET2);
		}
		~isoRes() {}
	};

	OriTmatrix::OriTmatrix()
		: qsca(0), qext(0), walb(0), time(0), g(0), nmax(0), qbk(0)
	{
		base = boost::shared_ptr<const tmatrixParams>(new tmatrixParams);
		//iso_angle_res = boost::shared_ptr<isoRes>(new isoRes);
	}

	boost::shared_ptr<const OriTmatrix> OriTmatrix::calc(
		boost::shared_ptr<const tmatrixParams> in)
	{
		boost::shared_ptr<OriTmatrix> res(new OriTmatrix);
		res->base = in;

		if (in->needApply())
		{
			in->apply();
			tmatrix_iso_double_();
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
		if (res->base->needApply())
		{
			res->base->apply();
			tmatrix_iso_double_();
		}
		res->import();
		res->calcParams();
		return res;
	}

	// TODO!

	void OriTmatrix::import()
	{
		using namespace std;
		double pi = boost::math::constants::pi<double>();
		double sizep = 2. * pi * this->base->axi / this->base->lam;
		double scale = 2. / (sizep * sizep);
		//double pArea = calcProjAreaAvg(this->base);
		double pArea = pi * pow(this->base->axi,2.);
		qsca = outputsiso_.CSCAT / pArea;
		qext = std::abs(outputsiso_.CEXTIN) / pArea;
		walb = std::abs(outputsiso_.WALB);
		time = outputsiso_.TIME;
		nmax = outputsiso_.NMAX;
		g = outputsiso_.ASYMM;
		qbk = outputsiso_.BK * outputsiso_.CSCAT / pArea;

		iso_angle_res = boost::shared_ptr<isoRes>(new isoRes(outputsiso_));

		int errcode = 0;
		errcode = outputsiso_.QERROR;
		if (errcode) throw_tmerror(errcode);
	}

	void OriTmatrix::calcParams() {}

	IsoAngleRes::IsoAngleRes() {}
	IsoAngleRes::~IsoAngleRes() {}
	bool IsoAngleRes::operator<(const IsoAngleRes &r) const {
		if (tm != r.tm) return tm < r.tm;
/*
		if (P.size() != r.P.size()) return P.size() < r.P.size();
		auto it = P.begin();
		auto ot = r.P.begin();
		while (it != P.end()) {
			if (it->first != ot->first) return it->first < ot->first;
			for (size_t i=0; i<16; ++i)
				if (it->second[i] != ot->second[i]) 
					return it->second[i] < ot->second[i];
		}
		*/
		return false;
	}

	boost::shared_ptr<const IsoAngleRes> IsoAngleRes::calc(
		boost::shared_ptr<const OriTmatrix> tma) {
		boost::shared_ptr<IsoAngleRes> res(new IsoAngleRes);
		res->tm = tma;
		/*
		//std::cerr << "ALPH1\tALPH2\tALPH3\tALPH4\tBET1\tBET2\n";
		//for (size_t i=0; i<tmatrix_random::defs::tmd::NPL; ++i) {
		//	std::cerr << outputsiso_.ALPH1[i] << "\t"
		//		<< outputsiso_.ALPH2[i] << "\t"
		//		<< outputsiso_.ALPH3[i] << "\t"
		//		<< outputsiso_.ALPH4[i] << "\t"
		//		<< outputsiso_.BET1[i] << "\t"
		//		<< outputsiso_.BET2[i] << std::endl;
		//}

		// Borrowed vaguely from Mishchenko code
		// Need the expansion coefficients ALPH1, ALPH2, ALPH3, ALPH4,
		// BET1, BET2, each of size defs::tmd::NPL.
		const size_t LMAX=defs::tmd::NPL;
		std::set<double> angles;
		auto popangles = [&](size_t numAngles) {
			angles.clear();
			for (size_t i=0; i < numAngles; ++i) {
				double a = 180. * ((double) i)/(numAngles-1.);
				angles.insert(a);
			}
		};
		// I default to every 10 degrees. See applyIso where this is set.
		popangles((size_t) inputsbiso_.NPNA );
		const double pi = boost::math::constants::pi<double>();
		for (const auto &ad : angles) { // angle in degrees
			double ar = ad * pi / 180.; // angle in radians
			double U = cos(ar);
			auto getIndex = [](size_t row, size_t col, bool startsatone) -> size_t {
				if (startsatone) { row--; col--; }
				size_t res = (4*row) + col; // note row-major vs. column-major
				return res;
			};
			std::array<double, 16> Fangle;
			double F11 = 0, F2 = 0, F3 = 0, F44=0,
				   F12=0, F34=0, F22=0, F33=0;
			std::array<double, 4> PP, P;
			PP[0] = 1; PP[1] = pow(1.+U,2.)/4.;
			PP[2] = pow(1.-U,2.)/4.; PP[3] = ((U*U)-1)*sqrt(6.)/4.;
			// Iterate over each element of the exxpansion coefficients
			for (size_t L=0; L<LMAX; ++L) {
				F11 += tma->iso_angle_res->data.ALPH1[L] * PP[0];
				F44 += tma->iso_angle_res->data.ALPH4[L] * PP[3];
				if (L != LMAX-1) {
					double PL1 = 1 + (2*L);
					double dP = ((PL1 * U * PP[0] ) - (L*P[0])) / (L+1.);
					P[0] = PP[0];
					PP[0] = dP;
					if (L >= 2) {
						F2 += (tma->iso_angle_res->data.ALPH2[L]+tma->iso_angle_res->data.ALPH3[L])*PP[1];
						F3 += (tma->iso_angle_res->data.ALPH2[L]-tma->iso_angle_res->data.ALPH3[L])*PP[2];
						F12 += tma->iso_angle_res->data.BET1[L] * PP[3];
						F34 += tma->iso_angle_res->data.BET2[L] * PP[3];
						if (L != LMAX-1) {
							double PL2 = U*L*(L+1.);
							double PL3 = ((L*L)-4.)*(L+1.);
							double PL4 = 1. / ((((L+1.)*(L+1.))-4.) * L);
							double dP = PL4 * ( ( (PL2-4.)*PL1*PP[1]) - (PL3*P[1]));
							P[1] = PP[1];
							PP[1] = dP;
							dP = PL4 * ( ( (PL2-4.)*PL1*PP[2]) - (PL3*P[2]));
							P[2] = PP[2];
							PP[2] = dP;
							dP = ((PL1*U*PP[3])-((sqrt((L*L)-4)*P[3])))/sqrt(pow(L+1.,2.)-4.);
							P[3] = PP[3];
							PP[3] = dP;
						}
					}
				}
				F22 = (F2+F3)/2.;
				F33 = (F2-F3)/2.;

				// Computation of F is done. Now store it.
				Fangle[getIndex(1,1,true)]=F11;
				Fangle[getIndex(2,2,true)]=F22;
				Fangle[getIndex(3,3,true)]=F33;
				Fangle[getIndex(4,4,true)]=F44;
				Fangle[getIndex(1,2,true)]=F12;
				Fangle[getIndex(3,4,true)]=F34;

				/// \todo TODO: Add in Mueller matrix symmetries here....
				res->P.insert(std::pair<double, std::array<double,16> >
					(ad, std::move(Fangle)));
			}
		}
		*/
		return res;
	}

	double getDifferentialBackscatterCrossSectionUnpol(
		boost::shared_ptr<const IsoAngleRes> angles) {
		// Get last angle
		if (!angles->P.count(180))
			RDthrow(::Ryan_Debug::error::xArrayOutOfBounds())
			<< ::Ryan_Debug::error::otherErrorText("Isotropic code did not calculate P at 180 degrees")
			<< ::Ryan_Debug::error::symbol_name("IsoAngleRes::P");

		double dBkSc = angles->P.at(180).at(0) * 4.0 / pow(angles->tm->base->axi,2.0);
		return dBkSc;
	}

	void printDebugInfo(std::ostream &out)
	{
		using std::cerr;
		using std::string;
		using std::endl;
		out << "Ryan_Tmatrix_Random information\nTODO\n";
		//	<< "Version: " << RYAN_TMATRIX_RANDOM_MAJOR << "."
		//	<< RYAN_TMATRIX_RANDOM_MINOR << "."
		//	<< RYAN_TMATRIX_RANDOM_REVISION << "."
		//	<< RYAN_TMATRIX_RANDOM_SVNREVISION << endl;
	}

}


