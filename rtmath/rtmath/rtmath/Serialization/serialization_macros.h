#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#define EXPORT(U,T) \
	template void U(boost::archive::text_oarchive &, T &, const unsigned int); \
	template void U(boost::archive::text_iarchive &, T &, const unsigned int); \
	template void U(boost::archive::xml_oarchive &, T &, const unsigned int); \
	template void U(boost::archive::xml_iarchive &, T &, const unsigned int);
