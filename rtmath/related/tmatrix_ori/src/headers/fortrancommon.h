#pragma once
#include <boost/cstdint.hpp>
#include "par.h"

     // COMMON /INPUTS/ AXI,RAT,LAM,MRR,MRI,EPS,NP,NDGS,
     //&    ALPHA,BETA,THET0,THET,PHI0,PHI,DDELT

extern "C"
{
	/*!
	\def DLIMPORT_TMATRIX_FORTRAN
	Indicates that the given symbol is imported from the
	Fortran library.
	*/
#ifdef _WIN32
#define DLIMPORT_TMATRIX_FORTRAN _declspec(dllimport)
#else
#define DLIMPORT_TMATRIX_FORTRAN
#endif
	extern DLIMPORT_TMATRIX_FORTRAN struct {
		double AXI,
				RAT,
				LAM,
				MRR,
				MRI,
				EPS,
				DDELT,
				ALPHA,
				BETA;
	} inputs_;

	extern DLIMPORT_TMATRIX_FORTRAN struct {
		double AXMAX, RAT, LAM, MRR, MRI, EPS, 
			   DDELT, GAM, B;
	} inputsiso_;

	extern DLIMPORT_TMATRIX_FORTRAN struct {
		boost::int32_t NP, NDGS;
	} inputsb_;

	extern DLIMPORT_TMATRIX_FORTRAN struct {
		boost::int32_t NP, NDGS, NDISTR, NKMAX, NPNA, NPNAX;
	} inputsbiso_;

	extern DLIMPORT_TMATRIX_FORTRAN struct {
		double THET0,
			THET,
			PHI0,
			PHI;
	} inputsang_;

	extern DLIMPORT_TMATRIX_FORTRAN struct {
		double QSCA;
		double QEXT;
		double WALB;
		double TIME;
		boost::int32_t NMAX;
		boost::int32_t QERROR;
	} outputs_;

	struct isores_internal {
		double CSCAT;
		double CEXTIN;
		double WALB;
		double TIME;
		boost::int32_t NMAX;
		boost::int32_t QERROR;
		double ASYMM;
		double ALPH1[tmatrix::defs::tmd::NPL],
			   ALPH2[tmatrix::defs::tmd::NPL],
			   ALPH3[tmatrix::defs::tmd::NPL],
			   ALPH4[tmatrix::defs::tmd::NPL],
			   BET1[tmatrix::defs::tmd::NPL],
			   BET2[tmatrix::defs::tmd::NPL];
	};
	extern DLIMPORT_TMATRIX_FORTRAN isores_internal outputsiso_;

	extern DLIMPORT_TMATRIX_FORTRAN struct {
		double OS[8];
		double OP[16];
		boost::int32_t AERROR;
	} outputsamp_;

	const size_t TSIZE = tmatrix::defs::ampld::NPN6 
		* tmatrix::defs::ampld::NPN4 * tmatrix::defs::ampld::NPN4;

	// These just had to be REAL*4 in Fortran.
	// In C++ storage, I'll use a 32-bit int type.
	/*
	extern DLIMPORT_TMATRIX_FORTRAN struct {
		boost::int32_t RT11[TSIZE];
		boost::int32_t RT12[TSIZE];
		boost::int32_t RT21[TSIZE];
		boost::int32_t RT22[TSIZE];
		boost::int32_t IT11[TSIZE];
		boost::int32_t IT12[TSIZE];
		boost::int32_t IT21[TSIZE];
		boost::int32_t IT22[TSIZE];
	} tmat_;
	*/
};

