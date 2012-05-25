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
	enum slots_s { T_DENS, DENS_T, DENS_M, DENS_V, M_DENS, V_DENS, V_REFF, REFF_V, NUMSLOTS };
	// Set signals
	enum signals_e { T = NUMSLOTS, DENS, M, V, REFF, NUMSIGSLOTS };

	//enum links_e { T, DENS, M, V, REFF, NUMLINKS };
	const char* links_n[] = { "T_DENS", "DENS_T", "DENS_M", "DENS_V",
		"M_DENS", "V_DENS", "V_REFF", "REFF_V",
		"T", "DENS", "M", "V", "REFF" };

	typedef std::pair<size_t, std::shared_ptr<const vertex> >  vPair;
	std::set<vPair> vertices;
	for (size_t i=0; i<NUMSLOTS; i++)
	{
		std::shared_ptr<vertex> np (new vertex);
		vertices.insert(vPair(i,np));
	}
	for (size_t i=NUMSLOTS; i<NUMSIGSLOTS; i++)
	{
		std::shared_ptr<vertex> np (new vertexVarProvided);
		vertices.insert(vPair(i,np));
	}

	// Add the links

	for (size_t i=0; i< NUMLINKS; i++)
	{
		std::set<std::shared_ptr<const vertex> > depends;
		switch (i)
		{
		case T:
			/*
			depends.insert(vertices[DENS]);
			vertices[T]->addRoot(depends);
			break;
			*/
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
	
	graph grp(sv);
	std::list< std::shared_ptr<const vertex> > order;
	std::set< std::shared_ptr<const vertex> > remaining;
	grp.generate(order, remaining);

	// Print output
	using namespace std;
	cout << "Solution:\n\t";
	for (auto it = order.begin(); it != order.end(); ++it)
		cout << links_n[(*it)->id()] << "\t";
	cout << endl << endl;
	cout << "Leftovers:\n\t";
	for (auto it = remaining.begin(); it != remaining.end(); it++)
		cout << links_n[(*it)->id()] << "\t";
	cout << endl << endl;
	return 0;
}