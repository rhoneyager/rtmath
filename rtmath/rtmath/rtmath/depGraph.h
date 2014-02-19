#pragma once
#include "defs.h"
#include <array>
//#include <list>
//#include <map>
//#include <memory>
#include <set>
#include <vector>
#include <unordered_set>
#include <cstdarg>
#ifdef __GNUC__
#include <initializer_list>
#endif
//#include <boost/shared_ptr.hpp>
//#include <boost/enable_shared_from_this.hpp>
//#include <boost/weak_ptr.hpp>

namespace rtmath
{
	namespace graphs
	{

		class graph;
		class vertex;

		class DLEXPORT_rtmath_core vertexRunnable
		{
		public:
			vertexRunnable() {}
			virtual ~vertexRunnable() {}
			// id field is present to allow for encapsulation within a class
			virtual void run(const std::string &id = "") = 0;
			// runSupported tells if the code can handle this id. Used for 
			// derived / base class vertexRunnable handling
			virtual bool runSupported(const std::string &id = "") = 0;
		};


		typedef std::unordered_set< vertex* > setVertex;
		/// \brief Provides vertex ordering when the graph is filled.
		/// \param First vertex* is the vertex
		/// \param size_t is the round in which the vertex was filled.
		/// \param Second vertex* is the parent vertex that was the 
		/// source for the filling (if an OR vertex). If null, then 
		/// there was no single parent.
		typedef std::vector< std::tuple<vertex*, size_t, vertex*> > orderedVertex;
		typedef std::array<vertex*, 80> stackVertex;


		class DLEXPORT_rtmath_core vertex // : public boost::enable_shared_from_this<vertex>
		{
		public:
			vertex(bool OR = false);
			virtual ~vertex();
			inline void setOR(bool o) { _slotOR = o; }
			inline bool isOR() const { return _slotOR; }
			bool addSlot(vertex* slot);
			void run(vertexRunnable*, const std::string &id = "") const;
			void run(const std::string &id = "") const;
			void setVertexRunnableCode(vertexRunnable* target);

			// Make connector between two criteria
			//			static std::shared_ptr<vertex> connect(
			//				std::shared_ptr<vertex> target, 
			//				size_t nDepends, ...); 
			static vertex* connect(
				vertex* target,
				const std::set<vertex* > &depends);
			// Fast connect that looks up node id (MANIPULATED_QUANTITY)
			//static std::shared_ptr<vertex> connect(
			//	size_t node, size_t numDeps, ...);
#ifdef __GNUC__
			static vertex* connect(
				vertex* target,
				std::initializer_list<vertex* > depends
				);
#endif

			stackVertex _signals, _slots;
			//boost::shared_ptr<vertexRunnable> _target;
			vertexRunnable* _target;
			bool _slotOR;
			bool _addSignal(vertex* signal);
			friend class graph;
		};


		/// \brief Convenient template function to get the number of objects in an 
		/// array, ending with the first null object.
		template <class T>
		size_t arraySizeToNull(const T &obj)
		{
			size_t res = 0;
			for (auto it = obj.cbegin(); it != obj.cend(); ++it)
			{
				if ((*it) == nullptr) break;
				res++;
			}
			return res;
		}

		/*
		/// \brief Convenient template function to get the number objects in an 
		/// array, ending with the first null object.
		template <class T>
		typename decltype(T)::const_iterator arrayNullCEnd(const T &obj)
		{
			auto it = obj.cbegin();
			for (; it != obj.cend(); ++it)
			{
				if ((*it) == nullptr) return it;
			}
			return obj.cend();
		}
		*/

		template<typename vertexSet = setVertex,
			typename orderedVertices = orderedVertex>
		class generateGraph
		{
		public:
			static void generate(
			const vertexSet &vertices,
			const vertexSet &provided,
			orderedVertices &_order,
			vertexSet &_remaining,
			vertexSet &ignored)
			{
				const size_t numVertices = vertices.size();
				std::map<vertex*, size_t> ranking;
				vertexSet _filled;
				//_filled.reserve(numVertices);
				_filled = provided; /// \todo Use std::copy
				//_filled.reserve(numVertices);

				// Do this way for shared/weak_ptr conversion
				for (auto it = vertices.begin(); it != vertices.end(); it++)
				{
					//std::cerr << "Adding vertex " << it->get() << "\n";
					_remaining.insert(*it);
				}

				// Remove provided from remaining
				for (auto it = provided.begin(); it != provided.end(); it++)
				{
					_remaining.erase(*it);
					ranking[*it] = 0;
				}

				//size_t pass = 1; // Records pass number in which vertices are filled
				// Loop each depth layer
				while (_remaining.size())
				{
					vertexSet cleanup;
					//cleanup.reserve(numVertices);
					size_t vertices_added = 0;

					// First, start with the vertices that have no roots or have a root that is filled
					// Remove these from _remaining, add to _filled and place them in the ordering
					for (auto it = _remaining.begin(); it != _remaining.end(); it++)
					{
						bool ready = false;
						vertex* parent = nullptr;
						//if (it->expired()) continue;
						auto IT = *it; //->lock();
						//std::cerr << "Checking " << IT.get() << " with " << IT->_slots.size() 
						//	<< " slots\n";
						if (!arraySizeToNull(IT->_slots)) ready = true;
						//if (!IT->_slots.size()) ready = true;

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
						for (auto ot = IT->_signals.cbegin(); ot != IT->_signals.cend(); ++ot, ++i)
						//for (auto ot = IT->_signals.cbegin(); ot != arrayNullCEnd(IT->_signals); ++ot, ++i)
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
							ignored.insert(*it);
							cleanup.insert(*it);
							continue;
						}

						// Check to see if a root is completely filled (hence ready for extraction)
						// Look at all root members to see if root is filled
						size_t n = arraySizeToNull(IT->_slots); // IT->_slots.size();
						size_t m = 0;
						size_t depth = 1;

						std::multimap<size_t, vertex*> parent_cands;

						for (auto ot = IT->_slots.begin(); ot != IT->_slots.end(); ot++)
						//for (auto ot = IT->_slots.cbegin(); ot != arrayNullCEnd(IT->_slots); ot++)
						{
							if (!(*ot)) continue;
							//if (ot->expired()) continue;
							if (_filled.count(*ot))
							{
								m++;

								size_t d = 0;
								if (ranking.count(*ot)) d = ranking.at(*ot);
								parent_cands.insert(std::pair<size_t, vertex*>(d, *ot));
							}
							// No longer doing this because of the correct depth search:
							//if (m && IT->_slotOR) break; 
						}
						// Depth search in case of multiple parents
						if (parent_cands.size())
						{
							auto f = parent_cands.begin();
							parent = f->second;
							depth = f->first+1;
						} else {
							parent = nullptr;
							depth = 0;
						}


						if (m && IT->_slotOR) ready = true;
						if (m == n) ready = true;

						// If ready, place in _order, _filled and remove from _remaining
						if (ready)
						{
							_order.push_back(std::tuple<vertex*, size_t, vertex*>
								(*it, depth, parent));
							ranking.insert(std::pair<vertex*, size_t>(*it, depth));
							_filled.insert(*it);
							cleanup.insert(*it);
							vertices_added++;
						}
					}

					//order++; // Increment depth count (for storage)

					// Cleanup loop (to erase elements)
					for (auto ct = cleanup.begin(); ct != cleanup.end(); ct++)
					{
						_remaining.erase(*ct);
					}

					// Check for isolates - these are unconnectable in any loop
					if ((!vertices_added) && _remaining.size()) break;
				}
			}
		};
	}
}

