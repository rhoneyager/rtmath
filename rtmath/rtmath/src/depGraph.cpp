#include "Stdafx-core.h"

#include <set>
#include <memory>
#ifdef __GNUC__
#include <initializer_list>
#endif

#include "../rtmath/depGraph.h"
#include "../rtmath/error/error.h"

namespace rtmath
{
	namespace graphs
	{
#ifdef __GNUC__
		vertex* vertex::connect(
			vertex* target, 
			std::initializer_list<vertex* > depends)
		{
			// Create connector
			vertex* connector;
			connector = vertex*(new vertex(false) );
			target->addSlot(connector);
			for (vertex* it : depends)
				connector->addSlot(it);

			return connector;
		}
#endif

		vertex* vertex::connect(
			vertex* target,
			const std::set<vertex* > &depends)
		{
			vertex* connector = new vertex(false);
			target->addSlot(connector);
			for (auto it = depends.begin(); it != depends.end(); it++)
			//for (std::shared_ptr<vertex> it : depends)
				connector->addSlot(*it);

			return connector;
		}

		vertex::vertex(bool OR) : _slotOR(OR), _target(nullptr)
		{
			//_signals.reserve(12);
			//_slots.reserve(12);
			std::fill(_slots.begin(), _slots.end(), nullptr);
			std::fill(_signals.begin(), _signals.end(), nullptr);
		}

		vertex::~vertex() {}

		void vertex::addSlot(vertex* slot)
		{
			// Find first unused _slots
			auto it = std::find_if(_slots.begin(), _slots.end(), 
				[](const vertex *v)
			{
				if (v) return false;
				return true;
			});
			//auto it = std::find(_slots.begin(), _slots.end(), nullptr);
			if (it == _slots.end()) RTthrow debug::xArrayOutOfBounds();
			*it = slot;
			//_slots.insert(slot);
			// Check if the parent is OR. If so, set the parent's signal
			if (!slot->_slotOR)
				slot->_addSignal( this );
		}

		void vertex::_addSignal(vertex* signal)
		{
			//_signals.insert(signal);

			auto it = std::find_if(_signals.begin(), _signals.end(), 
				[](const vertex *v)
			{
				if (v) return false;
				return true;
			});
			//auto it = std::find(_signals.begin(), _signals.end(), nullptr);
			if (it == _signals.end()) RTthrow debug::xArrayOutOfBounds();
			*it = signal;
		}

		void vertex::run(vertexRunnable *target, const std::string &id) const
		{
			if (target)
				target->run(id);
		}

		void vertex::run(const std::string &id) const
		{
			if (_target)
				_target->run(id);
		}

		void vertex::setVertexRunnableCode(vertexRunnable* target)
		{
			_target = target;
		}

		graph::graph(const setVertex &vertices)
		{
			_vertices = vertices;
			/*
			for (auto it = vertices.begin(); it != vertices.end(); it++)
				_vertices.insert(*it);
			*/
		}

		void graph::generate(const setVertex &provided, 
			orderedVertex &_order, 
			setVertex &_remaining,
			setVertex &_useless)
		{
			_order.clear();
			_order.reserve(_vertices.size());
			setVertex _filled;
			_filled.reserve(_vertices.size());
			_useless.clear();
			_useless.reserve(_vertices.size());
			// Do this way for shared/weak_ptr conversion
			for (auto it = _vertices.begin(); it != _vertices.end(); it++)
			{
				//std::cerr << "Adding vertex " << it->get() << "\n";
				_remaining.insert(*it);
			}
			//_remaining = _vertices;

			_filled = provided;
			// Remove provided from remaining
			for (auto it = provided.begin(); it != provided.end(); it++)
			{
				//std::cerr << "Vertex " << it->lock().get() << " is provided\n";
				_remaining.erase(*it);
			}

			size_t order = 1; // Records pass number in which vertices are filled
			// Loop each depth layer
			while (_remaining.size())
			{
				setVertex cleanup;
				cleanup.reserve(_vertices.size());
				size_t vertices_added = 0;

				// First, start with the vertices that have no roots or have a root that is filled
				// Remove these from _remaining, add to _filled and place them in the ordering
				for (auto it = _remaining.begin(); it != _remaining.end(); it++)
				{
					bool ready = false;
					//if (it->expired()) continue;
					auto IT = *it; //->lock();
					//std::cerr << "Checking " << IT.get() << " with " << IT->_slots.size() 
					//	<< " slots\n";
					if (!IT->_slots.size()) ready = true;

					// Check to see if signals exist and if they are filled
					auto hasSignal = std::find_if(IT->_signals.begin(), IT->_signals.end(), 
						[](const vertex *v)
					{
						if (v) return true;
						return false;
					});
					bool signalblock = (hasSignal == IT->_signals.end()) ? false : true;
					//bool signalblock = (IT->_signals.size()) ? true : false;
					//std::cerr << "\tHas " << IT->_signals.size() << " signals\n";
					int i = 0; // Used when debugging
					for (auto ot = IT->_signals.begin(); ot != IT->_signals.end(); ++ot, ++i)
					{
						if (!(*ot)) continue;
						if (!_filled.count(*ot))
						{
							//std::cerr << "\tSignal " << i << " " << ot->lock().get() << " not filled\n";
							signalblock = false;
							break;
						}
					}
					if (signalblock)
					{
						//std::cerr << "\tSignal is filled, so this vertex is unnecessary\n";
						_useless.insert(*it);
						cleanup.insert(*it);
						continue;
					}

					// Check to see if a root is completely filled (hence ready for extraction)
					// Look at all root members to see if root is filled
					size_t n = IT->_slots.size();
					size_t m = 0;

					for (auto ot = IT->_slots.begin(); ot != IT->_slots.end(); ot++)
					{
						if (!(*ot)) continue;
						//if (ot->expired()) continue;
						if (_filled.count(*ot)) m++;
						if (m && IT->_slotOR) break; // No need to go on
					}
					if (m && IT->_slotOR) ready = true;
					if (m == n) ready = true;

					// If ready, place in _order, _filled and remove from _remaining
					if (ready)
					{
						_order.push_back(std::pair<vertex*, size_t>
							(*it, order));
						_filled.insert(*it);
						cleanup.insert(*it);
						vertices_added++;
					}
				}

				order++; // Increment depth count (for storage)

				// Cleanup loop (to erase elements)
				for (auto ct = cleanup.begin(); ct != cleanup.end(); ct++)
				{
					_remaining.erase(*ct);
				}

				// Check for isolates - these are unconnectable in any loop
				if ((!vertices_added) && _remaining.size()) break;
			}
		}



	}
}

