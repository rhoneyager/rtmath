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
		typedef std::vector< std::pair<vertex*, size_t> > orderedVertex;
		typedef std::array<vertex*, 60> stackVertex;


		class DLEXPORT_rtmath_core vertex // : public boost::enable_shared_from_this<vertex>
		{
		public:
			vertex(bool OR = false);
			virtual ~vertex();
			inline void setOR(bool o) { _slotOR = o; }
			inline bool isOR() const { return _slotOR; }
			void addSlot(vertex* slot);
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

		protected:
			//boost::shared_ptr<vertexRunnable> _target;
			vertexRunnable* _target;
			bool _slotOR;
			void _addSignal(vertex* signal);
			stackVertex _signals, _slots;
			friend class graph;
		};


		class DLEXPORT_rtmath_core graph // : public boost::enable_shared_from_this<graph>
		{
		public:
			graph(const setVertex &vertices);
			void generate(const setVertex &provided, 
				orderedVertex &order, 
				setVertex &remaining,
				setVertex &ignored);
		private:
			setVertex _vertices; 
			//setVertex _remaining, _filled, _unfillable, _useless;
			//orderedVertex _order;
		};


	}
}

