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

#define EXPORTING_RTMATH

#include "../rtmath/mie/mie.h"

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include "../rtmath/Serialization/eigen_serialization.h"
//#include <boost/archive/text_oarchive.hpp>
//#include <boost/archive/text_iarchive.hpp>

#define EXPORT(U,T) \
	template void DLEXPORT_RTMATH U(boost::archive::xml_oarchive &, T &, const unsigned int); \
	template void DLEXPORT_RTMATH U(boost::archive::xml_iarchive &, T &, const unsigned int);
// 	template void U(boost::archive::text_oarchive &, T &, const unsigned int); 
//	template void U(boost::archive::text_iarchive &, T &, const unsigned int); 

//BOOST_STRONG_TYPEDEF(double, tDouble);

namespace boost
{
	namespace serialization
	{
		// NOTE: This is a duplicate function. Cannot do static linking
		template<class Archive> 
		void serialize(Archive &ar, tDouble & ti, const unsigned int version){ 
			// serialize the underlying int 
			ar & boost::serialization::make_nvp("val", static_cast<double &>(ti) ); 
		} 

		// NOTE: This is a duplicate function. Cannot do static linking
		template<class Archive> 
		void serialize(Archive &ar, tREAL & ti, const unsigned int version){ 
			// serialize the underlying int 
			ar & boost::serialization::make_nvp("val", static_cast<boost::int32_t &>(ti) ); 
		} 
		
		template <class Archive>
		void serialize(Archive & ar, rtmath::mie::mieParams & g, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("AXI", g.axi);
			ar & boost::serialization::make_nvp("LAM", g.lam);
			ar & boost::serialization::make_nvp("DDELT", g.ddelt);
			ar & boost::serialization::make_nvp("M", g.m);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::mie::mieCalc & g, const unsigned int version)
		{
			//ar & boost::serialization::make_nvp("base_params", g.base);
			boost::shared_ptr<rtmath::mie::mieParams> base = boost::const_pointer_cast
				<rtmath::mie::mieParams>(g.base);
			ar & boost::serialization::make_nvp("base_params", base);
			g.base = boost::shared_ptr<const rtmath::mie::mieParams>(base);

			ar & boost::serialization::make_nvp("QSCA", g.qsca);
			ar & boost::serialization::make_nvp("QEXT", g.qext);
			ar & boost::serialization::make_nvp("QABS", g.qabs);
			ar & boost::serialization::make_nvp("G", g.g);
			ar & boost::serialization::make_nvp("WALB", g.walb);
			ar & boost::serialization::make_nvp("SIZEP", g.sizep);
		}

		template <class Archive>
		void serialize(Archive & ar, rtmath::mie::mieAngleRes & g, const unsigned int version)
		{
			boost::shared_ptr< rtmath::mie::mieCalc > tm(
				boost::const_pointer_cast< rtmath::mie::mieCalc >(g.mc));
			ar & boost::serialization::make_nvp("mieCalc", tm);
			g.mc = boost::shared_ptr<const rtmath::mie::mieCalc>(tm);

			ar & boost::serialization::make_nvp("theta", g.theta);
			ar & boost::serialization::make_nvp("mu", g.mu);

			ar & boost::serialization::make_nvp("S", g.S);
			ar & boost::serialization::make_nvp("P", g.P);
		}


		EXPORT(serialize,rtmath::mie::mieParams);
		EXPORT(serialize,rtmath::mie::mieCalc);
		EXPORT(serialize,rtmath::mie::mieAngleRes);
	}
}

//BOOST_CLASS_VERSION(tmatrix::tmatrixInVars, 1)

