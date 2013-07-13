#include "../rtmath/Stdafx.h"
#include <Eigen/Core>
//#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/shapefile.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>

#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/Serialization/eigen_serialization.h"


namespace rtmath
{
	namespace ddscat
	{
		template<class Archive>
		void shapefile::serialize(Archive &ar, const unsigned int version)
		{
				ar & boost::serialization::make_nvp("Filename", filename);
				if (version) // Hash the shapefile for searching
					ar & boost::serialization::make_nvp("Hash", _localhash);
				ar & boost::serialization::make_nvp("Description", desc);
				ar & boost::serialization::make_nvp("N", numPoints);
				ar & boost::serialization::make_nvp("Dielectrics", Dielectrics);
				ar & boost::serialization::make_nvp("a1", a1);
				ar & boost::serialization::make_nvp("a2", a2);
				ar & boost::serialization::make_nvp("a3", a3);
				ar & boost::serialization::make_nvp("d", d);
				ar & boost::serialization::make_nvp("x0", x0);
				ar & boost::serialization::make_nvp("xd", xd);
/*
				if (version)
				{
					// Write out the points. Eigen's serialization routines should work well here.
					ar & boost::serialization::make_nvp("latticePts", latticePts);
					ar & boost::serialization::make_nvp("latticePtsStd", latticePtsStd);
					ar & boost::serialization::make_nvp("latticePtsNorm", latticePtsNorm);
					ar & boost::serialization::make_nvp("latticePtsRi", latticePtsRi);
					
					ar & boost::serialization::make_nvp("mins", mins);
					ar & boost::serialization::make_nvp("maxs", maxs);
					ar & boost::serialization::make_nvp("means", means);
				}
*/
		}

		EXPORTINTERNAL(rtmath::ddscat::shapefile::serialize);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapefile);

