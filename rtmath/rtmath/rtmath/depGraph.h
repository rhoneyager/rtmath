#pragma once
#include <list>
#include <map>
#include <memory>
#include <set>
#include <cstdarg>
#ifdef __GNUC__
#include <initializer_list>
#endif

namespace rtmath
{
	namespace graphs
	{

		class graph;
		class vertex;

		class vertexRunnable
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

		typedef std::set< std::weak_ptr<vertex>, std::owner_less<std::weak_ptr<vertex> > > setWeakVertex;
		typedef std::set< std::shared_ptr<vertex> > setShrdVertex;
		typedef std::list< std::weak_ptr<vertex> > listWeakVertex;

		class vertex : public std::enable_shared_from_this<vertex>
		{
		public:
			vertex(bool OR = false) : _slotOR(OR) { _target = nullptr; }
			virtual ~vertex();
			void addSlot(std::shared_ptr<vertex> slot);
			void run(std::shared_ptr<vertexRunnable> target) const;
			void run() const;
			void setVertexRunnableCode(std::shared_ptr<vertexRunnable> target);

			// Make connector between two criteria
			//			static std::shared_ptr<vertex> connect(
			//				std::shared_ptr<vertex> target, 
			//				size_t nDepends, ...); 
			static std::shared_ptr<vertex> connect(
				std::shared_ptr<vertex> target,
				const std::set<std::shared_ptr<vertex> > &depends);
			// Fast connect that looks up node id (MANIPULATED_QUANTITY)
			//static std::shared_ptr<vertex> connect(
			//	size_t node, size_t numDeps, ...);
#ifdef __GNUC__
			static std::shared_ptr<vertex> connect(
				std::shared_ptr<vertex> target,
				std::initializer_list<std::shared_ptr<vertex> > depends
				);
#endif

		protected:
			std::shared_ptr<vertexRunnable> _target;
			bool _slotOR;
			void _addSignal(std::weak_ptr<vertex> signal);
			setWeakVertex _signals, _slots;
			friend class graph;
		};


		class graph : public std::enable_shared_from_this<graph>
		{
		public:
			graph(const setShrdVertex &vertices);
			void generate(const setWeakVertex &provided, 
				listWeakVertex &order, 
				setWeakVertex &remaining,
				setWeakVertex &ignored);
		private:
			void _generate(const setWeakVertex &provided);
			setShrdVertex _vertices; 
			setWeakVertex _remaining, _filled, _unfillable, _useless;
			listWeakVertex _order;
		};


	}
}

