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

//#define EXPORTING_RTMATH

#include "../rtmath/mie/mie.h"

#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/Serialization/eigen_serialization.h"

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
		
	}
}

namespace rtmath
{
	namespace mie
	{
		template <class Archive>
		void mieParams::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("AXI", axi);
			ar & boost::serialization::make_nvp("LAM", lam);
			ar & boost::serialization::make_nvp("DDELT", ddelt);
			ar & boost::serialization::make_nvp("M", m);
		}

		template <class Archive>
		void mieCalc::serialize(Archive & ar, const unsigned int version)
		{
			//ar & boost::serialization::make_nvp("base_params", base);
			boost::shared_ptr<rtmath::mie::mieParams> base = boost::const_pointer_cast
				<rtmath::mie::mieParams>(this->base);
			ar & boost::serialization::make_nvp("base_params", base);
			this->base = boost::shared_ptr<const rtmath::mie::mieParams>(base);

			ar & boost::serialization::make_nvp("QSCA", qsca);
			ar & boost::serialization::make_nvp("QEXT", qext);
			ar & boost::serialization::make_nvp("QABS", qabs);
			ar & boost::serialization::make_nvp("G", g);
			ar & boost::serialization::make_nvp("WALB", walb);
			ar & boost::serialization::make_nvp("SIZEP", sizep);
		}

		template <class Archive>
		void mieAngleRes::serialize(Archive & ar, const unsigned int version)
		{
			boost::shared_ptr< rtmath::mie::mieCalc > tm(
				boost::const_pointer_cast< rtmath::mie::mieCalc >(mc));
			ar & boost::serialization::make_nvp("mieCalc", tm);
			mc = boost::shared_ptr<const rtmath::mie::mieCalc>(tm);

			ar & boost::serialization::make_nvp("theta", theta);
			ar & boost::serialization::make_nvp("mu", mu);

			ar & boost::serialization::make_nvp("S", S);
			ar & boost::serialization::make_nvp("P", P);
		}


		EXPORTINTERNAL(rtmath::mie::mieParams::serialize);
		EXPORTINTERNAL(rtmath::mie::mieCalc::serialize);
		EXPORTINTERNAL(rtmath::mie::mieAngleRes::serialize);
	}
}

//BOOST_CLASS_VERSION(tmatrix::tmatrixInVars, 1)

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::mie::mieParams);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::mie::mieCalc);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::mie::mieAngleRes);

