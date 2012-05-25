#pragma once


#include <memory>
#include <set>
#include <map>
#include <list>
#include <string>

class graph;

class vertex : public std::enable_shared_from_this<vertex>
{
public:
	vertex(size_t id = 0) : _id(id) {}
	void addRoot(const std::set< std::shared_ptr<const vertex> > & root);
	void dropRoots();
	inline size_t id() const { return _id; }
private:
	std::set< std::set< std::shared_ptr<const vertex> > > _roots;
	size_t _id;
	friend class graph;
};



class graph : public std::enable_shared_from_this<graph>
{
public:
	graph(const std::set< std::shared_ptr<const vertex> > &vertices);
	void generate(std::list< std::shared_ptr<const vertex> > &order, 
		std::set< std::shared_ptr<const vertex> > &remaining);
private:
	void _generate();
	std::set< std::shared_ptr<const vertex> > _vertices, _remaining, _filled, _unfillable;
	std::list< std::shared_ptr<const vertex> > _order;
};

