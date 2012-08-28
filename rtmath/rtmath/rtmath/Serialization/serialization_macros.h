#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#define EXPORT(T) \
	template <> void serialize(boost::archive::text_oarchive &, T &, const unsigned int); \
	template <> void serialize(boost::archive::text_iarchive &, T &, const unsigned int); \
	template <> void serialize(boost::archive::xml_oarchive &, T &, const unsigned int); \
	template <> void serialize(boost::archive::xml_iarchive &, T &, const unsigned int);

