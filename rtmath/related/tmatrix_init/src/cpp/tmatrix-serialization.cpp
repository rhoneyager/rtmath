#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/vector.hpp>
#include <vector>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
// #include <boost/serialization/strong_typedef.hpp> // duplicates boost/strong_typedef.hpp
#include <boost/serialization/complex.hpp>
#include <boost/serialization/set.hpp>
//#include <Ice/Ice.h>

#define EXPORTING_TMATRIX

#include "../headers/tmatrix.h"

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#define EXPORT(U,T) \
	template void DLEXPORT_TMATRIX U(boost::archive::xml_oarchive &, T &, const unsigned int); \
	template void DLEXPORT_TMATRIX U(boost::archive::xml_iarchive &, T &, const unsigned int); \
	template void DLEXPORT_TMATRIX U(boost::archive::text_oarchive &, T &, const unsigned int); \
	template void DLEXPORT_TMATRIX U(boost::archive::text_iarchive &, T &, const unsigned int);

// 	template void U(boost::archive::text_oarchive &, T &, const unsigned int); 
//	template void U(boost::archive::text_iarchive &, T &, const unsigned int); 

//BOOST_STRONG_TYPEDEF(double, tDouble);

namespace boost
{
	namespace serialization
	{
		template<class Archive> 
		void serialize(Archive &ar, tDouble & ti, const unsigned int version){ 
			// serialize the underlying int 
			ar & boost::serialization::make_nvp("val", static_cast<double &>(ti) ); 
		} 

		template<class Archive> 
		void serialize(Archive &ar, tREAL & ti, const unsigned int version){ 
			// serialize the underlying int 
			ar & boost::serialization::make_nvp("val", static_cast<boost::int32_t &>(ti) ); 
		} 
		
		template <class Archive>
		void serialize(Archive & ar, tmatrix::tmatrixParams & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("AXI", g.axi);
			ar & boost::serialization::make_nvp("RAT", g.rat);
			ar & boost::serialization::make_nvp("LAM", g.lam);
			ar & boost::serialization::make_nvp("EPS", g.eps);
			ar & boost::serialization::make_nvp("DDELT", g.ddelt);
			ar & boost::serialization::make_nvp("M", g.m);
			ar & boost::serialization::make_nvp("NP", g.np);
			ar & boost::serialization::make_nvp("NDGS", g.ndgs);
		}

		template <class Archive>
		void serialize(Archive & ar, tmatrix::OriTmatrix & g, const unsigned int version)
		{
			boost::shared_ptr< tmatrix::tmatrixParams > base(
				boost::const_pointer_cast<tmatrix::tmatrixParams>(g.base));
			ar & boost::serialization::make_nvp("base_params", base);
			g.base = boost::shared_ptr< const tmatrix::tmatrixParams > (base);
			// version 1+ have normalized cross-sections to ddscat raw cross-sections
			ar & boost::serialization::make_nvp("QSCA", g.qsca);
			ar & boost::serialization::make_nvp("QEXT", g.qext);
			ar & boost::serialization::make_nvp("WALB", g.walb);
			ar & boost::serialization::make_nvp("TIME", g.time);
			ar & boost::serialization::make_nvp("NMAX", g.nmax);
			ar & boost::serialization::make_nvp("ALPHA", g.alpha);
			ar & boost::serialization::make_nvp("BETA", g.beta);

			ar & boost::serialization::make_nvp("projArea", g.projArea);
		}

		template <class Archive>
		void serialize(Archive & ar, tmatrix::OriAngleRes & g, const unsigned int version)
		{
			boost::shared_ptr< tmatrix::OriTmatrix > tm(
				boost::const_pointer_cast< tmatrix::OriTmatrix >(g.tm));
			ar & boost::serialization::make_nvp("OriTmatrix", tm);
			g.tm = boost::shared_ptr< const tmatrix::OriTmatrix > (tm);

			ar & boost::serialization::make_nvp("theta", g.theta);
			ar & boost::serialization::make_nvp("theta0", g.theta0);
			ar & boost::serialization::make_nvp("phi", g.phi);
			ar & boost::serialization::make_nvp("phi0", g.phi0);

			boost::shared_ptr<std::vector<std::complex<tDouble> > > S(
				boost::const_pointer_cast<std::vector<std::complex<tDouble> > >(g.S));
			boost::shared_ptr<std::vector<tDouble> > P(
				boost::const_pointer_cast<std::vector<tDouble> >(g.P));
			ar & boost::serialization::make_nvp("S", S);
			ar & boost::serialization::make_nvp("P", P);
			g.S = boost::shared_ptr< const std::vector<std::complex<tDouble> > > (S);
			g.P = boost::shared_ptr< const std::vector<tDouble> > (P);
		}

		template <class Archive>
		void serialize(Archive & ar, tmatrix::queue::TM & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("tasks", g.tasks);
			ar & boost::serialization::make_nvp("responsesTM", g.responsesTM);
			ar & boost::serialization::make_nvp("responsesAngles", g.responsesAngles);
		}

		template <class Archive>
		void serialize(Archive & ar, tmatrix::queue::TMangles & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("theta", g.theta);
			ar & boost::serialization::make_nvp("theta0", g.theta0);
			ar & boost::serialization::make_nvp("phi", g.phi);
			ar & boost::serialization::make_nvp("phi0", g.phi0);
		}

		template <class Archive>
		void serialize(Archive & ar, tmatrix::queue::TMrequest & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("params", g.params);
			ar & boost::serialization::make_nvp("alpha", g.alpha);
			ar & boost::serialization::make_nvp("beta", g.beta);
			ar & boost::serialization::make_nvp("angles", g.angles);
		}

		EXPORT(serialize,tmatrix::tmatrixParams);
		EXPORT(serialize,tmatrix::OriTmatrix);
		EXPORT(serialize,tmatrix::OriAngleRes);
		EXPORT(serialize,tmatrix::queue::TM);
		EXPORT(serialize,tmatrix::queue::TMangles);
		EXPORT(serialize,tmatrix::queue::TMrequest);
	}
}

//BOOST_CLASS_VERSION(tmatrix::tmatrixInVars, 1)
BOOST_CLASS_VERSION(tmatrix::OriTmatrix, 1)

