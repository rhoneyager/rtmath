#pragma once
/* This header file will be included by apps, not the main part of the library, to include the 
 * necessary headers and link with the necessary libraries in the ROOT distribution.
 * The pragmas matter most on MSVC, which does not automatically include the libraries.
 * On Unix/Linux, this is unnecessary as these are listed in the Makefile build steps.
 */

#pragma warning( push )
#pragma warning( disable : 4996 )
#pragma warning( disable : 4800 )
#include <TGraph.h>
#include <TF1.h>
#include <TCanvas.h>
#include <TAxis.h>
#include <TNamed.h>
#include <TGraph2D.h>
#include <TStyle.h>
#include <TH2.h>
#include <TFile.h>
#include <TTree.h>
#include <TComplex.h>
#include <TObject.h>
#pragma warning( pop ) 


#pragma comment(lib, "libCore")
#pragma comment(lib, "libCint")
#pragma comment(lib, "libRIO")
#pragma comment(lib, "libNet")
#pragma comment(lib, "libHist")
#pragma comment(lib, "libGraf")
#pragma comment(lib, "libGraf3d")
#pragma comment(lib, "libGpad")
#pragma comment(lib, "libTree")
#pragma comment(lib, "libRint")
#pragma comment(lib, "libPostscript")
#pragma comment(lib, "libMatrix")
#pragma comment(lib, "libPhysics")
#pragma comment(lib, "libMathCore")
#pragma comment(lib, "libThread")

