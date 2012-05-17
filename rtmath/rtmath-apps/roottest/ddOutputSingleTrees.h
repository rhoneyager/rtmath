#pragma once
#include <string>
#include <complex>
#include <map>
#include <set>
#include "../../rtmath/rtmath/ROOTlink.h"


struct ddOutputSingleTreeBranch : public TObject
{
private:
	Double_t _freq;
	Double_t _size[4000];
	Double_t _size_P;
public:

	//std::set<ddOutputSingleTreeLeaf> outputSingle;
	ddOutputSingleTreeBranch()
	{
		_freq = 0; _size_P = 0;
		for (size_t i=0; i<4000; i++)
			_size[i] = 0;
	}

	void freq(Double_t nf) { _freq = nf; }
	Double_t freq() const { return _freq; }
	void sz(Double_t nf, size_t pos) { _size[pos] = nf; }
	Double_t sz(size_t pos) const { return _size[pos]; }
	void size_P(Double_t nf) { _size_P = nf; }
	Double_t size_P() const { return _size_P; }

	ClassDef(ddOutputSingleTreeBranch,1);
	//ClassImp(ddOutputSingleTreeBranch);
};
