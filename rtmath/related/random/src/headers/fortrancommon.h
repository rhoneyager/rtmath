#pragma once
#include <boost/cstdint.hpp>
#include "par.h"

     // COMMON /INPUTS/ AXI,RAT,LAM,MRR,MRI,EPS,NP,NDGS,
     //&    ALPHA,BETA,THET0,THET,PHI0,PHI,DDELT

extern "C"
{
	/*!
	\def DLIMPORT_TMATRIX_RANDOM_FORTRAN
	Indicates that the given symbol is imported from the
	Fortran library.
	*/
#ifdef _WIN32
#define DLIMPORT_TMATRIX_RANDOM_FORTRAN _declspec(dllimport)
#else
#define DLIMPORT_TMATRIX_RANDOM_FORTRAN
#endif

	extern DLIMPORT_TMATRIX_RANDOM_FORTRAN struct {
		double AXMAX, RAT, LAM, MRR, MRI, EPS,
			   DDELT, GAM, B;
	} inputsiso_;

	extern DLIMPORT_TMATRIX_RANDOM_FORTRAN struct {
		boost::int32_t NP, NDGS, NDISTR, NKMAX, NPNA, NPNAX, DBG;
	} inputsbiso_;

	struct isores_internal {
		double CSCAT;
		double CEXTIN;
		double WALB;
		double TIME;
		boost::int32_t NMAX;
		boost::int32_t QERROR;
		double ASYMM;
		double REFF;
		double VEFF;
		double BK;
		//double ALPH1[tmatrix_random::defs::tmd::NPL],
		//	   ALPH2[tmatrix_random::defs::tmd::NPL],
		//	   ALPH3[tmatrix_random::defs::tmd::NPL],
		//	   ALPH4[tmatrix_random::defs::tmd::NPL],
		//	   BET1[tmatrix_random::defs::tmd::NPL],
		//	   BET2[tmatrix_random::defs::tmd::NPL];
	};
	extern DLIMPORT_TMATRIX_RANDOM_FORTRAN isores_internal outputsiso_;

	const size_t TSIZE = tmatrix_random::defs::ampld::NPN6 
		* tmatrix_random::defs::ampld::NPN4 * tmatrix_random::defs::ampld::NPN4;

	// These just had to be REAL*4 in Fortran.
	// In C++ storage, I'll use a 32-bit int type.
	/*
	extern DLIMPORT_TMATRIX_RANDOM_FORTRAN struct {
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

