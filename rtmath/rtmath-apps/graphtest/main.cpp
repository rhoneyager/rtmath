/* this is a test of boost graph */
#include <iostream>
#include <utility> // std::pair
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <memory>
#include <set>
#include <map>
#include "../../rtmath/rtmath/rtmath.h"

#include "vertex2.h"

int main(int argc, char** argv)
{
	rtmath::debug::appEntry(argc, argv);

	// For the example, I'm setting up a variant of the standard topological sort. A
	// topological sort applies to any directed acyclic graph. In my variant, the incoming 
	// edges may be grouped together. If all other end vertices in a group are active, then the 
	// node also becomes active. I am not requiring a unique solution, either. I just want a 
	// solution that traverses the nodes in some near-minimum amount of steps.

	// Set up the structures
	// slots go in. signals get emitted (so out)
	// Set slots
	enum slots_s { T_DENS, DENS_T, DENS_V__M, DENS_M__V, M_V__DENS, V_REFF, REFF_V, NUMSLOTS };
	// Set signals
	enum signals_e { T = NUMSLOTS, DENS, M, V, REFF, NUMSIGSLOTS };

	//enum links_e { T, DENS, M, V, REFF, NUMLINKS };
	const char* links_n[] = { "T_DENS", "DENS_T", "DENS_V__M", "DENS_M__V",
		"M_V__DENS", "V_REFF", "REFF_V", "T","DENS", "M", "V", "REFF" };

	typedef std::pair<size_t, std::shared_ptr<vertex> >  vPair;
	std::set<std::shared_ptr< const vertex> > vertices;
	std::map<size_t, std::shared_ptr<vertex> > mVerts;
	for (size_t i=0; i<NUMSLOTS; i++)
	{
		std::shared_ptr<vertex> np (new vertex);
		np->id(i);
		vertices.insert(np);
		mVerts[i] = np;
	}
	for (size_t i=NUMSLOTS; i<NUMSIGSLOTS; i++)
	{
		std::shared_ptr<vertex> np (new vertexVarProvided);
		np->id(i);
		vertices.insert(np);
		mVerts[i] = np;
	}

	// Add the links
	mVerts[T_DENS]->addSignal(mVerts[DENS]);
	mVerts[DENS_T]->addSignal(mVerts[T]);
	mVerts[DENS_V__M]->addSignal(mVerts[M]);
	mVerts[DENS_M__V]->addSignal(mVerts[V]);
	mVerts[M_V__DENS]->addSignal(mVerts[DENS]);
	mVerts[V_REFF]->addSignal(mVerts[REFF]);
	mVerts[REFF_V]->addSignal(mVerts[V]);

	mVerts[DENS]->addSlot(mVerts[T_DENS]);
	mVerts[T]->addSlot(mVerts[DENS_T]);
	mVerts[M]->addSlot(mVerts[DENS_V__M]);
	mVerts[V]->addSlot(mVerts[DENS_M__V]);
	mVerts[DENS]->addSlot(mVerts[M_V__DENS]);
	mVerts[REFF]->addSlot(mVerts[V_REFF]);
	mVerts[V]->addSlot(mVerts[REFF_V]);

	// Check that these links are accurate...
	mVerts[T_DENS]->addSlot(mVerts[T]);
	mVerts[DENS_T]->addSlot(mVerts[DENS]);
	mVerts[DENS_V__M]->addSlot(mVerts[DENS]);
	mVerts[DENS_V__M]->addSlot(mVerts[V]);
	mVerts[DENS_M__V]->addSlot(mVerts[DENS]);
	mVerts[DENS_M__V]->addSlot(mVerts[M]);
	mVerts[M_V__DENS]->addSlot(mVerts[M]);
	mVerts[M_V__DENS]->addSlot(mVerts[V]);
	mVerts[V_REFF]->addSlot(mVerts[V]);
	mVerts[REFF_V]->addSlot(mVerts[REFF]);



	/*
	for (size_t i=0; i< NUMLINKS; i++)
	{
		std::set<std::shared_ptr<const vertex> > depends;
		switch (i)
		{
		case T:
			
			depends.insert(vertices[DENS]);
			vertices[T]->addRoot(depends);
			break;
			
		case DENS:
			depends.insert(vertices[T]);
			vertices[DENS]->addRoot(depends);
			depends.clear();
			depends.insert(vertices[M]);
			depends.insert(vertices[V]);
			vertices[DENS]->addRoot(depends);
			break;
		case M:
			depends.insert(vertices[V]);
			depends.insert(vertices[DENS]);
			vertices[M]->addRoot(depends);
			break;
		case V:
			depends.insert(vertices[REFF]);
			vertices[V]->addRoot(depends);
			depends.clear();
			depends.insert(vertices[M]);
			depends.insert(vertices[DENS]);
			vertices[V]->addRoot(depends);
			break;
		case REFF:
			
			depends.insert(vertices[V]);
			vertices[REFF]->addRoot(depends);
			
			break;
		default:
			break;
		}
	}
	*/

	std::set< std::shared_ptr<const vertex> > provided;
	provided.insert(mVerts[REFF]);
	provided.insert(mVerts[T]);

	graph grp(vertices);
	std::list< std::shared_ptr<const vertex> > order;
	std::set< std::shared_ptr<const vertex> > remaining, ignored;
	grp.generate(provided, order, remaining, ignored);

	// Print output
	using namespace std;
	cout << "Solution:\n\t";
	for (auto it = order.begin(); it != order.end(); ++it)
		cout << links_n[(*it)->id()] << "\t";
	cout << endl << endl;
	cout << "Leftovers:\n\t";
	for (auto it = remaining.begin(); it != remaining.end(); it++)
		cout << links_n[(*it)->id()] << "\t";
	for (auto it = ignored.begin(); it != ignored.end(); it++)
		cout << links_n[(*it)->id()] << "\t";
	cout << endl << endl;
	return 0;
}


