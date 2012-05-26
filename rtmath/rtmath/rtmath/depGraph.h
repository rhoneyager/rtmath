#pragma once
#include <list>
#include <map>
#include <memory>
#include <set>
#include <cstdarg>

namespace rtmath
{
	namespace graphs
	{

		class graph;
		class vertex;

		typedef std::set< std::weak_ptr<const vertex>, std::owner_less<std::weak_ptr<const vertex> > > setWeakVertex;
		//typedef std::list< std::weak_ptr<const vertex>, std::owner_less<std::weak_ptr<const vertex> > > listWeakVertex;
		typedef std::list< std::weak_ptr<const vertex> > listWeakVertex;

		class vertex : public std::enable_shared_from_this<vertex>
		{
		public:
			vertex(bool OR = false) : _slotOR(OR), _id(0) {}
			virtual ~vertex();
			void addSlot(std::shared_ptr<vertex> slot);
			inline size_t id() const { return _id; }
			void id(size_t newid) { _id = newid; }

			// Make connector between two criteria
			static std::shared_ptr<vertex> connect(
				std::shared_ptr<vertex> &connector, 
				std::shared_ptr<vertex> target, 
				size_t nDepends, ...); 
		protected:
			bool _slotOR;
			size_t _id;
			void _addSignal(std::weak_ptr<const vertex> signal);
			setWeakVertex _signals, _slots;
			friend class graph;
		};


		class graph : public std::enable_shared_from_this<graph>
		{
		public:
			graph(const std::set< std::shared_ptr<const vertex> > &vertices);
			void generate(const setWeakVertex &provided, 
				listWeakVertex &order, 
				setWeakVertex &remaining,
				setWeakVertex &ignored);
		private:
			void _generate(const setWeakVertex &provided);
			std::set< std::shared_ptr<const vertex> > _vertices; 
			setWeakVertex _remaining, _filled, _unfillable, _useless;
			listWeakVertex _order;
		};


	}
}

