#include "vertex.h"

void vertex::addRoot(const std::set< std::shared_ptr<const vertex> > & root)
{
	_roots.insert(root);
}

void vertex::dropRoots()
{
	_roots.clear();
}

graph::graph(const std::set< std::shared_ptr<const vertex> > &vertices)
{
	_vertices = vertices;
	_generate();
}

void graph::_generate()
{
	_order.clear();
	_filled.clear();
	_remaining = _vertices;

	size_t order = 0;
	// Loop each depth layer
	while (_remaining.size())
	{
		std::set< std::shared_ptr<const vertex> > cleanup;
		size_t vertices_added = 0;

		// First, start with the vertices that have no roots or have a root that is filled
		// Remove these from _remaining, add to _filled and place them in the ordering
		for (auto it = _remaining.begin(); it != _remaining.end(); it++)
		{
			bool ready = false;
			if ((*it)->_roots.size() == 0) ready = true;

			// Check to see if a root is completely filled (hence ready for extraction)
			for (auto ot = (*it)->_roots.begin(); ot != (*it)->_roots.end(); ++ot)
			{
				// Look at all root members to see if root is filled
				size_t n = ot->size();
				size_t m = 0;
				for (auto ut = ot->begin(); ut != ot->end(); ut++)
				{
					if (_filled.count(*ut)) m++;
				}
				if (m == n) ready = true;

				if (ready) break;
			}

			// If ready, place in _order, _filled and remove from _remaining
			if (ready)
			{
				_order.push_back(*it);
				_filled.insert(*it);
				cleanup.insert(*it);
				vertices_added++;
			}
		}


		// Cleanup loop (to erase elements)
		for (auto ct = cleanup.begin(); ct != cleanup.end(); ct++)
		{
			_remaining.erase(*ct);
		}

		// Check for isolates - these are unconnectable in any loop
		if ((!vertices_added) && _remaining.size()) break;
	}
}

void graph::generate(std::list< std::shared_ptr<const vertex> > &order, 
		std::set< std::shared_ptr<const vertex> > &remaining)
{
	_generate();
	order = _order;
	remaining = _remaining;
}


