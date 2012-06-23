#include "../rtmath/Stdafx.h"

#include <set>
#include <memory>
#ifdef __GNUC__
#include <initializer_list>
#endif


#include "../rtmath/error/error.h"
#include "../rtmath/depGraph.h"


namespace rtmath
{
	namespace graphs
	{
#ifdef __GNUC__
		std::shared_ptr<vertex> vertex::connect(
			std::shared_ptr<vertex> target, 
			std::initializer_list<std::shared_ptr<vertex> > depends)
		{
			// Create connector
			std::shared_ptr<vertex> connector;
			connector = std::shared_ptr<vertex>(new vertex(false) );
			target->addSlot(connector);
			for (std::shared_ptr<vertex> it : depends)
				connector->addSlot(it);

			return connector;
		}
#endif

		std::shared_ptr<vertex> vertex::connect(
			std::shared_ptr<vertex> target,
			const std::set<std::shared_ptr<vertex> > &depends)
		{
			std::shared_ptr<vertex> connector;
			connector = std::shared_ptr<vertex>(new vertex(false) );
			target->addSlot(connector);
			for (std::shared_ptr<vertex> it : depends)
				connector->addSlot(it);

			return connector;
		}
		/*
		std::shared_ptr<vertex> vertex::connect(
			size_t node, size_t numDeps, ...)
		{
			va_list indices;
			va_start(indices, numDeps);
			std::vector<std::shared_ptr<vertex> > ptr;
			std::shared_ptr<vertex> ival;
			for (size_t i=0; i<numDeps; i++)
			{
				ival = va_arg(indices, std::shared_ptr<vertex> );
				ptr.push_back(ival);
			}
			va_end(indices);

			// Create connector
			std::shared_ptr<vertex> connector;
			connector = std::shared_ptr<vertex>(new vertex(false) );
			target->addSlot(connector);
			for (auto it = ptr.begin(); it != ptr.end(); it++)
			{
				connector->addSlot((*it));
			}

			return connector;
		}
		*/
		vertex::~vertex() {}

		void vertex::addSlot(std::shared_ptr<vertex> slot)
		{
			_slots.insert(slot);
			// Check if the parent is OR. If so, set the parent's signal
			if (slot->_slotOR)
				slot->_addSignal( this->shared_from_this() );
		}

		void vertex::_addSignal(std::weak_ptr<vertex> signal)
		{
			_signals.insert(signal);
		}

		void vertex::run(std::shared_ptr<vertexRunnable> target) const
		{
			target->run();
		}

		void vertex::run() const
		{
			if (_target)
				_target->run();
		}

		void vertex::setVertexRunnableCode(std::shared_ptr<vertexRunnable> target)
		{
			_target = target;
		}

		graph::graph(const std::set< std::shared_ptr<vertex> > &vertices)
		{
			for (auto it = vertices.begin(); it != vertices.end(); it++)
				_vertices.insert(*it);
		}

		void graph::_generate(const setWeakVertex &provided)
		{
			_order.clear();
			_filled.clear();
			_useless.clear();
			// Do this way for shared/weak_ptr conversion
			for (auto it = _vertices.begin(); it != _vertices.end(); it++)
				_remaining.insert(*it);
			//_remaining = _vertices;

			_filled = provided;
			// Remove provided from remaining
			for (auto it = provided.begin(); it != provided.end(); it++)
				_remaining.erase(*it);

			size_t order = 0;
			// Loop each depth layer
			while (_remaining.size())
			{
				setWeakVertex cleanup;
				size_t vertices_added = 0;

				// First, start with the vertices that have no roots or have a root that is filled
				// Remove these from _remaining, add to _filled and place them in the ordering
				for (auto it = _remaining.begin(); it != _remaining.end(); it++)
				{
					bool ready = false;
					if (it->expired()) continue;
					auto IT = it->lock();
					if (!IT->_slots.size()) ready = true;

					// Check to see if signals exist and if they are filled
					bool signalblock = (IT->_signals.size()) ? true : false;
					for (auto ot = IT->_signals.begin(); ot != IT->_signals.end(); ot++)
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
					size_t n = IT->_slots.size();
					size_t m = 0;

					for (auto ot = IT->_slots.begin(); ot != IT->_slots.end(); ot++)
					{
						if (ot->expired()) continue;
						if (_filled.count(*ot)) m++;
						if (m && IT->_slotOR) break; // No need to go on
					}
					if (m && IT->_slotOR) ready = true;
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

		void graph::generate(const setWeakVertex &provided, 
			listWeakVertex &order, 
			setWeakVertex &remaining,
			setWeakVertex &ignored)
		{
			_generate(provided);
			order = _order;
			remaining = _remaining;
			ignored = _useless;
		}


	}
}

