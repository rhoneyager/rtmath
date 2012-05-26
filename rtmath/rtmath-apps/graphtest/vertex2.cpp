#include "vertex2.h"

vertex::~vertex() {}

void vertex::addSlot(std::shared_ptr<const vertex> slot)
{
	_slots.insert(slot);
}

void vertex::addSignal(std::shared_ptr<const vertex> signal)
{
	_signals.insert(signal);
}

graph::graph(const std::set< std::shared_ptr<const vertex> > &vertices)
{
	_vertices = vertices;
}

void graph::_generate(const std::set< std::shared_ptr<const vertex> > &provided)
{
	_order.clear();
	_filled.clear();
	_useless.clear();
	_remaining = _vertices;

	_filled = provided;
	// Remove provided from remaining
	for (auto it = provided.begin(); it != provided.end(); it++)
		_remaining.erase(*it);

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
			if (!(*it)->_slots.size()) ready = true;

			// Check to see if signals exist and if they are filled
			bool signalblock = ((*it)->_signals.size()) ? true : false;
			for (auto ot = (*it)->_signals.begin(); ot != (*it)->_signals.end(); ot++)
			{
				if (!_filled.count(*ot))
				{
					signalblock = false;
					break;
				}
			}
			if (signalblock)
			{
				_useless.insert(*it);
				cleanup.insert(*it);
				continue;
			}

			// Check to see if a root is completely filled (hence ready for extraction)
			// Look at all root members to see if root is filled
			size_t n = (*it)->_slots.size();
			size_t m = 0;
			for (auto ot = (*it)->_slots.begin(); ot != (*it)->_slots.end(); ot++)
			{
				if (_filled.count(*ot)) m++;
				if (m && (*it)->_slotOR) break; // No need to go on
			}
			if (m && (*it)->_slotOR) ready = true;
			if (m == n) ready = true;

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

void graph::generate(const std::set< std::shared_ptr<const vertex> > &provided,
		std::list< std::shared_ptr<const vertex> > &order, 
		std::set< std::shared_ptr<const vertex> > &remaining,
		std::set< std::shared_ptr<const vertex> > &ignored)
{
	_generate(provided);
	order = _order;
	remaining = _remaining;
	ignored = _useless;
}
