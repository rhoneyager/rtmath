#pragma once


#include <memory>
#include <set>
#include <map>
#include <list>
#include <string>

class graph;

// Here's my nutty logic: 
// During graph CONSTRUCTION
// If a vertex has no signals, it is ON
// If a vertex has any slots during graph-level eval, if they are all ON, the vertex is OFF
// If a vertex has any or all signals ON, see if vertex is AND or OR and respond appropriately

class vertex : public std::enable_shared_from_this<vertex>
{
public:
	vertex() : _slotOR(false), _id(0) {}
	virtual ~vertex();
	void addSlot(std::shared_ptr<const vertex> slot);
	void addSignal(std::shared_ptr<const vertex> signal);

	inline size_t id() const { return _id; }
	void id(size_t newid) { _id = newid; }
protected:
	bool _slotOR;
	size_t _id;
	std::set< std::shared_ptr<const vertex> > _signals, _slots;
	friend class graph;
};

class vertexVarProvided : public vertex
{
public:
	vertexVarProvided() { _slotOR = true; }
	virtual ~vertexVarProvided() {}
};

class graph : public std::enable_shared_from_this<graph>
{
public:
	graph(const std::set< std::shared_ptr<const vertex> > &vertices);
	void generate(const std::set< std::shared_ptr<const vertex> > &provided, 
		std::list< std::shared_ptr<const vertex> > &order, 
		std::set< std::shared_ptr<const vertex> > &remaining,
		std::set< std::shared_ptr<const vertex> > &ignored);
private:
	void _generate(const std::set< std::shared_ptr<const vertex> > &provided);
	std::set< std::shared_ptr<const vertex> > _vertices, _remaining, _filled, _unfillable, _useless;
	std::list< std::shared_ptr<const vertex> > _order;
	// TODO:	add functions to check for set intersection, union, ....
	//			Base it on theo. Used to determine if operation can complete.
};

