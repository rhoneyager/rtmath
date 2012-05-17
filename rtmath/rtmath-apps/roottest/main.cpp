#include "../../rtmath/rtmath/rtmath.h"
#include "../../rtmath/rtmath/ROOTlink.h"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/exponential_distribution.hpp>

#include "ddOutputSingleTrees.h"

int main(int argc, char** argv)
{
	using namespace std;
	cerr << "roottest" << endl;
	rtmath::debug::appEntry(argc, argv);

	TFile f("tree4.root","RECREATE");
	TTree t4("t4","A Tree with Events");

	boost::random::mt19937 rng; 
	boost::random::exponential_distribution<> gen(3.0);
	

	ddOutputSingleTreeBranch *a = new ddOutputSingleTreeBranch();
	a->freq(5);
	for (size_t i=0; i<4000; i++)
		a->sz(	gen(rng) , i);
	a->size_P(8);

	t4.Branch("event_branch1", "ddOutputSingleTreeBranch", &a,16000,0);
	t4.Branch("event_branch2", "ddOutputSingleTreeBranch", &a,16000,2);

	t4.Fill();      // Fill the tree

	f.Write();
	t4.Print();

	
	f.Print();
	//f.Close();
	//delete a;
	return 0;
}

